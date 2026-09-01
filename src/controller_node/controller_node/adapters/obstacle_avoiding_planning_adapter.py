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

Solo mira el segmento del TIP -- un obstáculo puede seguir chocando con el
codo/antebrazo aunque el tip lo esquive (ver ROADMAP.md, Bloque 4:
"Geometría del robot completo, no solo el tip"). Para eso, ver
`whole_body_obstacle_avoiding_planning_adapter.py`, que sí mira cada
eslabón.

Requiere que el `KinematicsPort` recibido exponga también
`forward_kinematics` (hoy solo `PoeKinematicsAdapter` lo hace) -- sin eso no
hay forma de saber dónde está el efector en cartesiano para comprobar si el
segmento pasa cerca del obstáculo. `NaivePlanningAdapter` no tiene esta
limitación porque no necesita saber dónde está nada; este adaptador sí.

Qué TIPO de planificador es esto, con precisión (pregunta real que surgió
analizándolo): dos ejes distintos, no uno.
  - CÓMO decide: no busca -- no explora alternativas ni construye un
    árbol/grafo (RRT), no tiene garantía de completitud ni optimalidad
    (CHOMP de verdad). Es una corrección local: "si la recta choca, empuja
    el punto justo en la dirección contraria al obstáculo" -- en espíritu,
    un campo de potenciales (Khatib 1986) de UN SOLO PASO: atracción al
    goal + repulsión del obstáculo, calculada analíticamente una vez, no
    recalculada como fuerza continua.
  - CUÁNDO decide: aquí NO es reactivo en tiempo real -- la `Scene` se da
    entera y fija de antemano, se calcula la trayectoria COMPLETA antes de
    mandar el primer waypoint (planificar una vez, ejecutar en bucle
    abierto). El propio ROADMAP.md ya llama a este bloque "Planificador
    reactivo con evitación (Régimen 2, rápido)" -- pero la parte
    verdaderamente reactiva-en-tiempo-real de esa idea ("Replanificación
    local cuando cambia el campo de obstáculos") sigue sin implementarse;
    esto es solo la heurística local, no el bucle cerrado.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

from shared_kernel import JointConfiguration, Pose, Scene, Trajectory

from ._segment_geometry import detour_point, worst_intersection


class _KinematicsPortWithForward(Protocol):
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose: ...


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
        # 1. Dónde está el tip AHORA MISMO en cartesiano -- forward_kinematics,
        # no algo que sepamos de antemano (current_configuration solo trae
        # ángulos de articulación).
        start_pose = self._kinematics.forward_kinematics(current_configuration)
        start = np.array([start_pose.x, start_pose.y, start_pose.z])
        goal_xyz = np.array([goal.x, goal.y, goal.z])

        # 2. ¿La línea recta start->goal invade algún obstáculo? Si no hay
        # ninguno (o ninguno lo bastante cerca), vía libre: IK directa al
        # goal, sin más -- este planificador se limita a ser una envoltura
        # transparente en el caso fácil.
        hit = worst_intersection(start, goal_xyz, scene.obstacles, self._clearance)
        if hit is None:
            return self._kinematics.compute_trajectory(goal, current_configuration)

        # 3. Hay colisión: calcular UN punto de paso que rodee al obstáculo
        # que peor invade, conservando la orientación del goal (no se
        # interpola orientación, es una simplificación deliberada).
        obstacle, center = hit
        via_xyz = detour_point(start, goal_xyz, obstacle, center, self._clearance)
        via_pose = Pose(
            x=float(via_xyz[0]),
            y=float(via_xyz[1]),
            z=float(via_xyz[2]),
            qx=goal.qx,
            qy=goal.qy,
            qz=goal.qz,
            qw=goal.qw,
        )

        # 4. Dos tramos, cada uno resuelto por el KinematicsPort recibido
        # (nunca por este planificador): primero hasta el punto de paso,
        # luego desde ahí hasta el goal real -- y se concatenan los
        # waypoints sin duplicar el de unión (to_goal.waypoints[1:] se
        # salta el primero, que es el mismo que el último de to_via).
        to_via = self._kinematics.compute_trajectory(via_pose, current_configuration)
        via_configuration = to_via.waypoints[-1]
        to_goal = self._kinematics.compute_trajectory(goal, via_configuration)
        waypoints = to_via.waypoints + to_goal.waypoints[1:]
        return Trajectory.create(waypoints).value
