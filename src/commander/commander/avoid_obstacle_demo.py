"""Demo de experimentación (rama `experimento/planificador-evita-obstaculo`,
ver ROADMAP.md Bloque 4 y Bloque 9): compone `KinematicsPort`
(`PoeKinematicsAdapter`, cinemática propia vía PoE) + `PlanningPort`
(`ObstacleAvoidingPlanningAdapter`, evitación mínima de un obstáculo
esférico) + `RobotControllerPort` (`CoppeliaSimRobotAdapter`) directamente,
sin pasar por `ControlSession`/`ControllerNode` -- `PlanningPort` todavía no
está wireado dentro del grafo de nodos ROS2 (ver
docs/pipeline_percepcion_planificacion.md §6). No necesita `rclpy`.

SEGUNDA versión de esta rama: ya NO depende de `cr5_base.ttt` (el `.ttt`
hecho a mano del que veníamos, con los problemas de arranque/estabilidad
que daba y una postura de reposo que no coincidía con la "home" matemática
de `PoeKinematicsAdapter`). En su lugar, `coppeliasim_scene_builder`
construye la escena por código a partir de una descripción -- qué robot
(CR5, importado de su URDF real), en qué postura inicial, qué `Scene` de
dominio (obstáculos) -- y la ejecuta contra una instancia de CoppeliaSim en
blanco. Efecto colateral importante: al importar el URDF sin recentrar el
modelo, `base_link_respondable` queda exactamente en el origen del mundo,
así que el marco interno de `PoeKinematicsAdapter` (relativo a `base_link`)
coincide con el marco mundo de CoppeliaSim POR CONSTRUCCIÓN -- ya no hace
falta ninguna calibración de marco (la que tenía la primera versión de este
demo, ver historial de la rama).

Postura inicial ("descripción"): CR5 en posición base (los seis joints a
cero -- la home real de `PoeKinematicsAdapter`, no una postura arbitraria de
un `.ttt`). Para probar con un joint distinto (p. ej. "90° en joint2"),
basta con cambiar `_INITIAL_CONFIGURATION` más abajo.

Goal y obstáculo, medidos con `PoeKinematicsAdapter.forward_kinematics`
desde esa postura base real (ya no hacen falta desde un valor "medido a
mano" contra una escena externa):
  - Home: (0, -0.246, 1.047).
  - Goal: mismo eje de orientación, movido a (0.25, 0.25, 0.50) --
    convergió numéricamente (distancia ~0.78 m, comprobado antes de
    fijarlo aquí).
  - Obstáculo: esfera de radio 0.08 m centrada en el punto medio real de
    ese segmento, (0.125, 0.002, 0.773).

Uso: `ros2 run commander avoid_obstacle_demo` (lanza CoppeliaSim solo si
hace falta). Revisa en la ventana de CoppeliaSim que la fila de dummies
azules (waypoints) rodee la esfera naranja (obstáculo) y termine en el
dummy rojo (objetivo).
"""

from __future__ import annotations

import time
from typing import Callable, Protocol

from controller_node.adapters.obstacle_avoiding_planning_adapter import (
    ObstacleAvoidingPlanningAdapter,
)
from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import (
    JointConfiguration,
    JointPosition,
    Point,
    Pose,
    Scene,
    SphereObstacle,
    Trajectory,
)

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000

# "Descripción" de la postura inicial -- posición base (todo a cero). Para
# probar "90 grados en joint2", por ejemplo:
#   JointConfiguration.create(
#       [JointPosition("joint1", 0.0), JointPosition("joint2", math.pi / 2), ...]
#   ).value
_INITIAL_CONFIGURATION = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value

_GOAL = Pose(
    x=0.25, y=0.25, z=0.50,
    qx=0.0, qy=-0.70711, qz=0.70711, qw=0.0,
)
_OBSTACLE = SphereObstacle(center=Point(0.125, 0.002, 0.773), radius=0.08)
_CLEARANCE = 0.05
_WAYPOINT_PAUSE_SECONDS = 0.3


class _PlanningPort(Protocol):
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration, scene: Scene
    ) -> Trajectory: ...


def _default_planner(kinematics: PoeKinematicsAdapter, clearance: float) -> _PlanningPort:
    return ObstacleAvoidingPlanningAdapter(kinematics, clearance=clearance)


def run(
    initial_configuration: JointConfiguration,
    goal: Pose,
    obstacle: SphereObstacle,
    clearance: float = _CLEARANCE,
    port: int = _ZMQ_PORT,
    planner_factory: Callable[[PoeKinematicsAdapter, float], _PlanningPort] = _default_planner,
) -> None:
    """Cuerpo reutilizable del demo: construye la escena para la
    `initial_configuration`/`obstacle` dadas, planifica hacia `goal`
    evitándolo, y ejecuta la trayectoria contra CoppeliaSim real. Permite
    definir otros ejemplos (otra postura inicial, otro obstáculo/goal, u
    otro `PlanningPort` vía `planner_factory`) sin duplicar la
    orquestación -- ver `avoid_obstacle_demo_joint2_90.py` (postura
    distinta) y `avoid_obstacle_demo_whole_body.py` (otro planificador)."""
    # Sufijo por puerto (no fijo): dos instancias simultáneas en puertos
    # distintos (ver avoid_obstacle_demo_compare.py) necesitan carpetas de
    # settings distintas, o compiten por el mismo usrset.txt y ninguna
    # termina de arrancar -- mismo hallazgo que ya documenta
    # coppeliasim_launcher.py para two_sessions_demo.py.
    ensure_coppeliasim_running(port=port, settings_suffix=f"_avoid_obstacle_demo_{port}")

    scene = Scene.empty().with_obstacle(obstacle)
    robot = build_cr5_scene(
        port=port,
        initial_configuration=initial_configuration,
        scene=scene,
    )
    robot.mark_goal(goal)

    kinematics = PoeKinematicsAdapter()
    planner = planner_factory(kinematics, clearance)

    trajectory = planner.compute_trajectory(goal, initial_configuration, scene)
    print(
        f"Trayectoria calculada: {len(trajectory.waypoints)} waypoints "
        f"(evitando {len(scene.obstacles)} obstáculo(s))"
    )

    for waypoint in trajectory.waypoints:
        robot.set_joints(waypoint)
        time.sleep(_WAYPOINT_PAUSE_SECONDS)

    print(
        "Listo. En CoppeliaSim: los dummies azules (waypoints) deberían "
        "rodear la esfera naranja (obstáculo) y el último debería coincidir "
        "con el dummy rojo (objetivo)."
    )


def main() -> None:
    run(_INITIAL_CONFIGURATION, _GOAL, _OBSTACLE)


if __name__ == "__main__":
    main()
