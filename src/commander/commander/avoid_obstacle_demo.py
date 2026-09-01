"""Demo de experimentación (rama `experimento/planificador-evita-obstaculo`,
ver ROADMAP.md Bloque 4): compone `KinematicsPort` (`PoeKinematicsAdapter`,
cinemática propia vía PoE) + `PlanningPort`
(`ObstacleAvoidingPlanningAdapter`, evitación mínima de un obstáculo
esférico) + `RobotControllerPort` (`CoppeliaSimRobotAdapter`) directamente,
sin pasar por `ControlSession`/`ControllerNode` -- `PlanningPort` todavía no
está wireado dentro del grafo de nodos ROS2 (ver
docs/pipeline_percepcion_planificacion.md §6), así que no hay ningún topic
nuevo que crear para probar esto de verdad. No necesita `rclpy`: es
composición de puertos en Python puro, igual que un test, salvo que aquí el
`RobotControllerPort` es CoppeliaSim de verdad en vez de un stub.

Goal y obstáculo están definidos en el marco INTERNO de `PoeKinematicsAdapter`
(el que exige `KinematicsPort`/`PlanningPort` -- ver `two_sessions_demo.py`):
  - AVISO real de esta rama: la configuración de reposo con la que arranca
    `cr5_base.ttt` (joint3=joint4=-pi/2, resto 0) NO es la misma que la
    configuración "home" (todos los ángulos a 0) que usa
    `PoeKinematicsAdapter`/`_DEFAULT_CR5_DESCRIPTION` como referencia para
    derivar los twists -- son dos cosas distintas que es fácil confundir (lo
    hice yo mismo en la primera versión de este demo).
  - Postura de reposo real (medida vía `forward_kinematics`, marco interno):
    (0.357, -0.246, 0.458), orientación ~90° sobre X.
  - Goal: mismo eje de orientación, movido a (0.20, 0.20, 0.35) -- convergió
    numéricamente desde esa postura real (distancia ~0.485 m, comprobado
    antes de fijarlo aquí).
  - Obstáculo: esfera de radio 0.08 m centrada en el punto medio real de
    ese segmento, (0.2785, -0.023, 0.404).

SEGUNDO hallazgo real de esta rama, más de fondo: el marco interno de
`PoeKinematicsAdapter` tampoco coincide con el marco MUNDO de CoppeliaSim
(mismo "hallazgo sin arreglar" que ya señala `two_sessions_demo.py`) -- si se
dibujan `_GOAL`/`_OBSTACLE` tal cual con `mark_goal`/`mark_obstacle`
(`setObjectPosition` en mundo), el robot llega de verdad al objetivo pero los
marcadores aparecen en otro sitio, dando la falsa impresión de que "no
llega". Medido con `Link6_visual`: es una transformación RÍGIDA CONSTANTE
(no depende de la configuración -- verificado prediciendo la matriz de
`Link6_visual` en una segunda postura a partir de la primera, error
~1e-16 m), así que se puede calibrar una vez por sesión con una sola
correspondencia (postura actual real vs. `forward_kinematics` de esa misma
postura) y aplicarse a cualquier Pose interna antes de dibujarla. El
planificador y la cinemática siguen operando enteramente en el marco
interno -- la calibración es puramente para que los marcadores no mientan.

Uso: `ros2 run commander avoid_obstacle_demo` (lanza CoppeliaSim solo si
hace falta, ver `coppeliasim_launcher`). Revisa en la ventana de CoppeliaSim
que la fila de dummies azules (waypoints, marco mundo real) rodee la esfera
naranja (obstáculo, ahora también en marco mundo) y termine en el dummy rojo.
"""

from __future__ import annotations

import os
import time

import numpy as np

from controller_node.adapters.obstacle_avoiding_planning_adapter import (
    ObstacleAvoidingPlanningAdapter,
)
from controller_node.adapters.poe_adapter import (
    PoeKinematicsAdapter,
    _matrix_to_pose,
    _pose_to_matrix,
)
from robot_node.adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter
from shared_kernel import JointConfiguration, Point, Pose, Scene, SphereObstacle

from .coppeliasim_launcher import ensure_coppeliasim_scene

_SCENE_PATH = os.path.expanduser("~/RoboticInvest/CoppeliaSim/scenes/cr5_base.ttt")
_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_TIP_NAME = "Link6_visual"
_ZMQ_PORT = 23000

# Marco interno de PoeKinematicsAdapter (ver docstring del módulo) -- NO son
# coordenadas de mundo de CoppeliaSim, se transforman con _calibrate() abajo.
_GOAL = Pose(
    x=0.20, y=0.20, z=0.35,
    qx=0.70711, qy=0.0, qz=0.0, qw=0.70711,
)
_OBSTACLE = SphereObstacle(center=Point(0.2785, -0.023, 0.404), radius=0.08)
_CLEARANCE = 0.05
_WAYPOINT_PAUSE_SECONDS = 0.3


def _calibrate(
    robot: CoppeliaSimRobotAdapter,
    kinematics: PoeKinematicsAdapter,
    current_configuration: JointConfiguration,
) -> np.ndarray:
    """Transformación rígida mundo<-interno: `world = T @ interno` (ver
    hallazgo en el docstring del módulo). Se calibra con UNA
    correspondencia -- la postura actual real, leída dos veces: por
    `forward_kinematics` (marco interno) y por `Link6_visual` (marco
    mundo) -- porque `T` es constante, no depende de la configuración."""
    raw = robot.get_tip_world_matrix()
    if raw is None:
        raise RuntimeError(
            "_calibrate: robot sin tip_name -- no hay forma de leer la "
            "pose real del tip en mundo (ver CoppeliaSimRobotAdapter.__init__)."
        )
    world = np.eye(4)
    world[0, :] = raw[0:4]
    world[1, :] = raw[4:8]
    world[2, :] = raw[8:12]
    internal = _pose_to_matrix(kinematics.forward_kinematics(current_configuration))
    return world @ np.linalg.inv(internal)


def _pose_to_world(transform: np.ndarray, pose: Pose) -> Pose:
    return _matrix_to_pose(transform @ _pose_to_matrix(pose))


def _obstacle_to_world(transform: np.ndarray, obstacle: SphereObstacle) -> SphereObstacle:
    # El radio es invariante bajo una transformación rígida (sin escalado) --
    # solo hace falta transformar el centro.
    center_pose = Pose(x=obstacle.center.x, y=obstacle.center.y, z=obstacle.center.z)
    world_center = _pose_to_world(transform, center_pose)
    return SphereObstacle(
        center=Point(world_center.x, world_center.y, world_center.z),
        radius=obstacle.radius,
    )


def main() -> None:
    ensure_coppeliasim_scene(
        port=_ZMQ_PORT,
        settings_suffix="_avoid_obstacle_demo",
        scene_path=_SCENE_PATH,
    )
    robot = CoppeliaSimRobotAdapter(
        joint_names=_JOINT_NAMES, tip_name=_TIP_NAME, zmq_port=_ZMQ_PORT
    )
    kinematics = PoeKinematicsAdapter()
    current_configuration = robot.get_current_configuration()

    transform = _calibrate(robot, kinematics, current_configuration)
    robot.mark_goal(_pose_to_world(transform, _GOAL))
    robot.mark_obstacle(_obstacle_to_world(transform, _OBSTACLE))

    # El planificador sigue operando enteramente en el marco interno -- la
    # calibración de arriba es solo para que los marcadores no mientan.
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=_CLEARANCE)
    scene = Scene.empty().with_obstacle(_OBSTACLE)

    trajectory = planner.compute_trajectory(_GOAL, current_configuration, scene)
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


if __name__ == "__main__":
    main()
