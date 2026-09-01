"""Segundo planificador mínimo de `PlanningPort` (shared_kernel), rama
`experimento/planificador-evita-obstaculo` (ver ROADMAP.md, Bloque 4:
"Geometría del robot completo, no solo el tip"): a diferencia de
`ObstacleAvoidingPlanningAdapter` (que solo comprueba el segmento recto del
TIP), este también comprueba que NINGÚN eslabón del robot -- el segmento
entre cada par de articulaciones consecutivas, y entre `base_link` y la
primera -- invada ningún `SphereObstacle`, en NINGÚN waypoint de la
trayectoria resultante. Usa `KinematicsPort.link_poses` (hoy solo
`PoeKinematicsAdapter` lo ofrece) para saber dónde está CADA articulación
en cada paso, no solo el tip.

Sigue siendo un planificador MÍNIMO, no CHOMP/RRT: no hay gradiente ni
optimización sobre la postura completa, y solo hay un lever de control real
(el objetivo cartesiano del TIP, vía `KinematicsPort.compute_trajectory`) --
no se puede pedir directamente "mueve el codo aquí". La estrategia es una
extensión iterativa de la de `ObstacleAvoidingPlanningAdapter`: si el
cuerpo completo invade algún obstáculo en algún waypoint, calcula un punto
de paso para el TIP que rodee al obstáculo peor invadido (igual que el otro
planificador) y, si el cuerpo SIGUE invadiendo algo tras eso, repite con
más margen -- hasta que el cuerpo entero quede libre o se agoten los
intentos, caso en el que devuelve el mejor intento sin garantía formal.

HALLAZGO real (visto en vivo contra CoppeliaSim, ver
avoid_obstacle_demo_whole_body.py): al encadenar varias llamadas a
`compute_trajectory` (cada intento de desvío hace dos, y cada una parte de
donde terminó la anterior), Newton-Raphson puede converger a un ángulo
matemáticamente válido (misma pose módulo 2π) pero MUY alejado del rango
±2π -- p. ej. joint4=-457°, joint6=540°, vistos en un caso real. Como
`JointConfiguration` no lleva ningún límite físico (ver
`shared_kernel/robot_description.py`: `JointDescription` no tiene
`limits`), CoppeliaSim (que sí tiene los joints limitados) recorta esos
ángulos en silencio -- la trayectoria "convergía" pero el robot real
acababa en una postura distinta a la calculada, sin ningún error. Mientras
no exista una noción real de límites por articulación en el dominio,
`_within_a_full_turn` (más abajo) actúa de salvaguarda genérica: rechaza
cualquier candidato cuyo ángulo bruto pase de ±2π, tratándolo igual que una
IK que no converge -- ver ROADMAP.md, Bloque 9, tarea pendiente de
verdad: límites de articulación en `RobotDescription`.
"""

from __future__ import annotations

import math
from typing import List, Optional, Protocol, Tuple

import numpy as np

from shared_kernel import JointConfiguration, Pose, Scene, SphereObstacle, Trajectory

from ._segment_geometry import closest_point_on_segment, detour_point, worst_intersection


class _KinematicsPortWithLinkPoses(Protocol):
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]: ...


def _within_a_full_turn(trajectory: Trajectory) -> bool:
    """Salvaguarda genérica frente al hallazgo del docstring del módulo:
    ningún ángulo articular de ningún waypoint debe pasar de ±2π en valor
    absoluto -- una IK que converge a una vuelta y media (p. ej. 540°) da
    la misma pose matemáticamente, pero un joint real (o, aquí, el mismo
    límite que ya trae el joint importado en CoppeliaSim) la recorta,
    dejando al robot en una postura distinta a la calculada."""
    for waypoint in trajectory.waypoints:
        for position in waypoint.positions:
            if abs(position.angle_radians) > 2 * math.pi:
                return False
    return True


def _body_segments(link_poses: List[Pose]) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Las 'aristas' del robot en una postura dada: `base_link` (siempre el
    origen en el marco de `KinematicsPort`) -> primera articulación -> ...
    -> tip (última pose de `link_poses`)."""
    points = [np.zeros(3)] + [np.array([p.x, p.y, p.z]) for p in link_poses]
    return list(zip(points[:-1], points[1:]))


class WholeBodyObstacleAvoidingPlanningAdapter:
    def __init__(
        self,
        kinematics: _KinematicsPortWithLinkPoses,
        clearance: float = 0.05,
        max_detour_attempts: int = 5,
        detour_growth_factor: float = 1.5,
    ) -> None:
        self._kinematics = kinematics
        self._clearance = clearance
        self._max_detour_attempts = max_detour_attempts
        self._detour_growth_factor = detour_growth_factor

    def compute_trajectory(
        self,
        goal: Pose,
        current_configuration: JointConfiguration,
        scene: Scene,
    ) -> Trajectory:
        trajectory = self._kinematics.compute_trajectory(goal, current_configuration)
        if (
            self._worst_body_hit(trajectory, scene.obstacles) is None
            and _within_a_full_turn(trajectory)
        ):
            return trajectory

        start_pose = self._kinematics.link_poses(current_configuration)[-1]
        start = np.array([start_pose.x, start_pose.y, start_pose.z])
        goal_xyz = np.array([goal.x, goal.y, goal.z])

        push_margin = self._clearance
        for _ in range(self._max_detour_attempts):
            hit = self._worst_body_hit(trajectory, scene.obstacles)
            if hit is None:
                break
            obstacle, center = hit
            via_xyz = detour_point(start, goal_xyz, obstacle, center, push_margin)
            via_pose = Pose(
                x=float(via_xyz[0]),
                y=float(via_xyz[1]),
                z=float(via_xyz[2]),
                qx=goal.qx,
                qy=goal.qy,
                qz=goal.qz,
                qw=goal.qw,
            )
            try:
                to_via = self._kinematics.compute_trajectory(
                    via_pose, current_configuration
                )
                via_configuration = to_via.waypoints[-1]
                to_goal = self._kinematics.compute_trajectory(goal, via_configuration)
            except RuntimeError:
                # El via-point de este intento no es alcanzable (IK no
                # convergió, ver KinematicsPort.compute_trajectory) -- no
                # es un fallo del planificador, solo un candidato inválido.
                # Nos quedamos con `trajectory` tal como estaba y probamos
                # el siguiente margen; si nunca converge ninguno, se agotan
                # los intentos y se devuelve el mejor candidato que sí
                # convergió (o la trayectoria directa si ninguno lo hizo).
                push_margin *= self._detour_growth_factor
                continue
            candidate = Trajectory.create(to_via.waypoints + to_goal.waypoints[1:]).value
            if not _within_a_full_turn(candidate):
                # Mismo motivo que el RuntimeError de arriba: Newton-Raphson
                # convergió a un ángulo válido en pose pero irrealizable
                # (ver hallazgo del docstring del módulo) -- se descarta
                # como candidato, no se acepta como si fuera correcto.
                push_margin *= self._detour_growth_factor
                continue
            trajectory = candidate
            if self._worst_body_hit(trajectory, scene.obstacles) is None:
                return trajectory
            push_margin *= self._detour_growth_factor

        # Se agotaron los intentos: el mejor candidato encontrado, aunque
        # no haya garantía de que el cuerpo entero quede libre -- ver
        # docstring del módulo, no es CHOMP/RRT.
        return trajectory

    def _worst_body_hit(
        self, trajectory: Trajectory, obstacles: List[SphereObstacle]
    ) -> Optional[Tuple[SphereObstacle, np.ndarray]]:
        """La peor intersección obstáculo-eslabón en CUALQUIER waypoint de
        `trajectory` -- recorre todos los waypoints y, en cada uno, todos
        los segmentos del cuerpo (`_body_segments`), no solo el del tip."""
        worst: Optional[Tuple[SphereObstacle, np.ndarray]] = None
        worst_penetration = 0.0
        for waypoint in trajectory.waypoints:
            link_poses = self._kinematics.link_poses(waypoint)
            for start, end in _body_segments(link_poses):
                hit = worst_intersection(start, end, obstacles, self._clearance)
                if hit is None:
                    continue
                obstacle, center = hit
                closest = closest_point_on_segment(start, end, center)
                distance = float(np.linalg.norm(closest - center))
                penetration = obstacle.radius + self._clearance - distance
                if worst is None or penetration > worst_penetration:
                    worst = hit
                    worst_penetration = penetration
        return worst
