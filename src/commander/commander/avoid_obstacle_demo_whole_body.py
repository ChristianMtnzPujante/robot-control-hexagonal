"""Tercer ejemplo de `avoid_obstacle_demo`: misma orquestación
(`avoid_obstacle_demo.run`), pero con `WholeBodyObstacleAvoidingPlanningAdapter`
en vez de `ObstacleAvoidingPlanningAdapter` (vía `planner_factory`) --
demuestra un caso real donde el planificador que solo mira el tip NO
detecta nada, pero el robot completo sí colisionaría.

Postura inicial: CR5 en posición base (todos los joints a cero).

Goal: el mismo que `avoid_obstacle_demo.py`, (0.25, 0.25, 0.50).

Obstáculo: esfera de radio 0.03 m en (0.25, 0.355, 0.384) -- colocada a
propósito cerca de donde queda `joint4` (el antebrazo) al llegar al goal,
NO cerca de la línea recta que sigue el tip (a 0.156 m de esa recta, más
lejos que radius+clearance=0.06). Verificado en local antes de fijarlo
aquí:
  - `ObstacleAvoidingPlanningAdapter` (tip-only) no lo detecta -- devuelve
    la trayectoria directa sin desviarse (21 waypoints).
  - `WholeBodyObstacleAvoidingPlanningAdapter` sí lo detecta (el segmento
    entre joint3 y joint4 invade el obstáculo) y encuentra un desvío que
    deja el cuerpo entero libre (41 waypoints, todos los ángulos finales
    dentro de ±2π -- ver el hallazgo real documentado en
    `whole_body_obstacle_avoiding_planning_adapter.py`: con radios/márgenes
    algo mayores, el primer desvío que "resolvía" matemáticamente pedía
    -457°/540° en joint4/joint6, que CoppeliaSim recorta en silencio; con
    este radio, el planificador converge dentro del rango físico real).

Uso: `ros2 run commander avoid_obstacle_demo_whole_body`.
"""

from __future__ import annotations

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from controller_node.adapters.whole_body_obstacle_avoiding_planning_adapter import (
    WholeBodyObstacleAvoidingPlanningAdapter,
)
from shared_kernel import JointConfiguration, JointPosition, Point, Pose, SphereObstacle

from .avoid_obstacle_demo import run

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

_INITIAL_CONFIGURATION = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value

_GOAL = Pose(
    x=0.25, y=0.25, z=0.50,
    qx=0.0, qy=-0.70711, qz=0.70711, qw=0.0,
)
_OBSTACLE = SphereObstacle(center=Point(0.25, 0.355, 0.384), radius=0.03)
_CLEARANCE = 0.03


def _whole_body_planner(
    kinematics: PoeKinematicsAdapter, clearance: float
) -> WholeBodyObstacleAvoidingPlanningAdapter:
    return WholeBodyObstacleAvoidingPlanningAdapter(
        kinematics, clearance=clearance, max_detour_attempts=15
    )


def main() -> None:
    run(
        _INITIAL_CONFIGURATION,
        _GOAL,
        _OBSTACLE,
        clearance=_CLEARANCE,
        planner_factory=_whole_body_planner,
    )


if __name__ == "__main__":
    main()
