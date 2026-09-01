"""Demo comparativo: lanza DOS instancias de CoppeliaSim a la vez (puertos
23000/23001), con el MISMO obstáculo/goal/postura inicial (el escenario de
`avoid_obstacle_demo_whole_body.py`), pero cada una resuelta por un
planificador distinto -- `ObstacleAvoidingPlanningAdapter` (tip-only) en
la primera, `WholeBodyObstacleAvoidingPlanningAdapter` (cuerpo completo)
en la segunda -- para comparar a ojo el rastro de waypoints (dummies
azules) que deja cada uno alrededor del mismo obstáculo naranja.

En este escenario el obstáculo está fuera de la línea recta que sigue el
tip (el planificador tip-only no se desvía, lo atraviesa de largo sin
enterarse) pero justo en el camino del antebrazo real (el planificador de
cuerpo completo sí se desvía) -- ver `avoid_obstacle_demo_whole_body.py`
para los números exactos ya verificados.

Uso: `ros2 run commander avoid_obstacle_demo_compare`. Las dos ventanas de
CoppeliaSim quedan abiertas al terminar -- compáralas: en la de la
izquierda (puerto 23000, tip-only) el rastro azul pasa cerca del
obstáculo; en la de la derecha (puerto 23001, cuerpo completo) se ve el
desvío.
"""

from __future__ import annotations

from controller_node.adapters.obstacle_avoiding_planning_adapter import (
    ObstacleAvoidingPlanningAdapter,
)
from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from controller_node.adapters.whole_body_obstacle_avoiding_planning_adapter import (
    WholeBodyObstacleAvoidingPlanningAdapter,
)

from .avoid_obstacle_demo import run
from .avoid_obstacle_demo_whole_body import (
    _CLEARANCE,
    _GOAL,
    _INITIAL_CONFIGURATION,
    _OBSTACLE,
)

_PORT_TIP_ONLY = 23000
_PORT_WHOLE_BODY = 23001


def _tip_only_planner(
    kinematics: PoeKinematicsAdapter, clearance: float
) -> ObstacleAvoidingPlanningAdapter:
    return ObstacleAvoidingPlanningAdapter(kinematics, clearance=clearance)


def _whole_body_planner(
    kinematics: PoeKinematicsAdapter, clearance: float
) -> WholeBodyObstacleAvoidingPlanningAdapter:
    return WholeBodyObstacleAvoidingPlanningAdapter(
        kinematics, clearance=clearance, max_detour_attempts=15
    )


def main() -> None:
    print(f"=== Sesión A (puerto {_PORT_TIP_ONLY}): planificador tip-only ===")
    run(
        _INITIAL_CONFIGURATION,
        _GOAL,
        _OBSTACLE,
        clearance=_CLEARANCE,
        port=_PORT_TIP_ONLY,
        planner_factory=_tip_only_planner,
    )

    print()
    print(f"=== Sesión B (puerto {_PORT_WHOLE_BODY}): planificador de cuerpo completo ===")
    run(
        _INITIAL_CONFIGURATION,
        _GOAL,
        _OBSTACLE,
        clearance=_CLEARANCE,
        port=_PORT_WHOLE_BODY,
        planner_factory=_whole_body_planner,
    )

    print()
    print(
        "Compara las dos ventanas de CoppeliaSim: en la del puerto "
        f"{_PORT_TIP_ONLY} (tip-only) el rastro azul pasa cerca de la esfera "
        f"naranja; en la del puerto {_PORT_WHOLE_BODY} (cuerpo completo) el "
        "antebrazo se desvía visiblemente para rodearla."
    )


if __name__ == "__main__":
    main()
