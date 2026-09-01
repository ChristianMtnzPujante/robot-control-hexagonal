"""Planificador mínimo de `PlanningPort` (shared_kernel): evita un único
`SphereObstacle` desviando la línea recta cartesiana con UN punto
intermedio, y delega en el `KinematicsPort` recibido para resolver cada
tramo -- no reimplementa IK ni interpolación en espacio de articulaciones,
solo decide POR DÓNDE debe pasar el efector. Experimento de la rama
`experimento/planificador-evita-obstaculo` (ver ROADMAP.md, Bloque 4): NO es
CHOMP ni RRT -- no hay gradiente, no hay optimización, no maneja varios
obstáculos a la vez (si la `Scene` trae varios, solo se esquiva el que más
invade el segmento recto). Objetivo: la evitación mínima que se pueda ver
funcionando de verdad en CoppeliaSim, antes de invertir en un planificador
serio.

Requiere que el `KinematicsPort` recibido exponga también
`forward_kinematics` (hoy solo `PoeKinematicsAdapter` lo hace) -- sin eso no
hay forma de saber dónde está el efector en cartesiano para comprobar si el
segmento pasa cerca del obstáculo. `NaivePlanningAdapter` no tiene esta
limitación porque no necesita saber dónde está nada; este adaptador sí.
"""

from __future__ import annotations

from typing import List, Optional, Protocol, Tuple

import numpy as np

from shared_kernel import (
    JointConfiguration,
    Pose,
    Scene,
    SphereObstacle,
    Trajectory,
)


class _KinematicsPortWithForward(Protocol):
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose: ...


def _closest_point_on_segment(
    start: np.ndarray, end: np.ndarray, point: np.ndarray
) -> np.ndarray:
    segment = end - start
    segment_length_sq = float(segment @ segment)
    if segment_length_sq < 1e-12:
        return start
    t = float(np.clip((point - start) @ segment / segment_length_sq, 0.0, 1.0))
    return start + t * segment


def _lateral_direction(segment: np.ndarray) -> np.ndarray:
    """Dirección perpendicular al segmento para rodear el obstáculo cuando
    el centro cae exactamente sobre la recta (caso degenerado: el vector
    "lejos del centro" tiene norma ~0). `segmento × arriba_del_mundo` da un
    lateral horizontal; si el segmento ya es vertical, cae a +X."""
    world_up = np.array([0.0, 0.0, 1.0])
    lateral = np.cross(segment, world_up)
    norm = np.linalg.norm(lateral)
    if norm < 1e-6:
        return np.array([1.0, 0.0, 0.0])
    return lateral / norm


def _worst_intersection(
    start: np.ndarray, goal: np.ndarray, obstacles: List[SphereObstacle], clearance: float
) -> Optional[Tuple[SphereObstacle, np.ndarray]]:
    """De entre los obstáculos que invaden el segmento start->goal (a menos
    de radius+clearance), el que más lo invade -- ese es el único que este
    planificador mínimo esquiva (ver docstring del módulo)."""
    worst: Optional[Tuple[SphereObstacle, np.ndarray, float]] = None
    for obstacle in obstacles:
        center = np.array([obstacle.center.x, obstacle.center.y, obstacle.center.z])
        closest = _closest_point_on_segment(start, goal, center)
        distance = float(np.linalg.norm(closest - center))
        required = obstacle.radius + clearance
        penetration = required - distance
        if penetration <= 0:
            continue
        if worst is None or penetration > worst[2]:
            worst = (obstacle, center, penetration)
    if worst is None:
        return None
    return worst[0], worst[1]


def _detour_point(
    start: np.ndarray, goal: np.ndarray, obstacle: SphereObstacle, center: np.ndarray, clearance: float
) -> np.ndarray:
    closest = _closest_point_on_segment(start, goal, center)
    away = closest - center
    distance = float(np.linalg.norm(away))
    required = obstacle.radius + clearance
    if distance > 1e-6:
        direction = away / distance
    else:
        direction = _lateral_direction(goal - start)
    return center + direction * required


class ObstacleAvoidingPlanningAdapter:
    def __init__(
        self, kinematics: _KinematicsPortWithForward, clearance: float = 0.05
    ) -> None:
        self._kinematics = kinematics
        self._clearance = clearance

    def compute_trajectory(
        self,
        goal: Pose,
        current_configuration: JointConfiguration,
        scene: Scene,
    ) -> Trajectory:
        start_pose = self._kinematics.forward_kinematics(current_configuration)
        start = np.array([start_pose.x, start_pose.y, start_pose.z])
        goal_xyz = np.array([goal.x, goal.y, goal.z])

        hit = _worst_intersection(start, goal_xyz, scene.obstacles, self._clearance)
        if hit is None:
            return self._kinematics.compute_trajectory(goal, current_configuration)

        obstacle, center = hit
        via_xyz = _detour_point(start, goal_xyz, obstacle, center, self._clearance)
        via_pose = Pose(
            x=float(via_xyz[0]),
            y=float(via_xyz[1]),
            z=float(via_xyz[2]),
            qx=goal.qx,
            qy=goal.qy,
            qz=goal.qz,
            qw=goal.qw,
        )

        to_via = self._kinematics.compute_trajectory(via_pose, current_configuration)
        via_configuration = to_via.waypoints[-1]
        to_goal = self._kinematics.compute_trajectory(goal, via_configuration)
        waypoints = to_via.waypoints + to_goal.waypoints[1:]
        return Trajectory.create(waypoints).value
