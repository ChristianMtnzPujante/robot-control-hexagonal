"""Demo del pseudo-perceptor: demuestra replanificación de verdad —"el
plan cambia porque cambió el mundo", no porque se recalcule a mano — en
la forma más mínima posible.

Secuencia:
  1. Se calcula una trayectoria hacia el goal SIN saber que hay un
     obstáculo (la `Scene` del `PseudoPerceptionAdapter` empieza vacía).
  2. Se envían los primeros waypoints -- el brazo ya se está moviendo.
  3. A mitad de camino, `PseudoPerceptionAdapter.report_obstacle(...)`
     simula una "detección" -- el evento que en el futuro vendría de una
     cámara real (ver ROADMAP.md, Bloque 3).
  4. Se replanifica desde la configuración ACTUAL del robot (no desde el
     inicio) hacia el mismo goal, ahora con el obstáculo en la `Scene` --
     y se envían los waypoints nuevos.

Es la versión más mínima posible de "Replanificación local cuando cambia
el campo de obstáculos" (ROADMAP.md, Bloque 4, todavía pendiente en su
forma general): aquí el "cambio" lo dispara una llamada de Python en este
mismo script, no un sensor real ni un topic ROS2 -- ver el spike de ciclo
de vida ya resuelto (Vikunja #89): el pseudo-perceptor tiene vida propia,
este demo se limita a ESCUCHARLO, igual que haría `Commander` más adelante.

Goal/obstáculo/punto de detección, verificados en local antes de fijarlos
aquí (ver commit): con el obstáculo en (0.28, 0.28, 0.42) radio 0.03,
detectado justo tras el waypoint 5 de 20, la replanificación converge
limpia -- cuerpo entero libre, ángulos dentro de ±2π, error de pose
~3 micrómetros.

Uso: `ros2 run commander perception_replan_demo`.
"""

from __future__ import annotations

import time

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from controller_node.adapters.whole_body_obstacle_avoiding_planning_adapter import (
    WholeBodyObstacleAvoidingPlanningAdapter,
)
from perception_node.adapters.pseudo_perception_adapter import PseudoPerceptionAdapter
from shared_kernel import JointConfiguration, JointPosition, Point, Pose, Scene, SphereObstacle

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000

_INITIAL_CONFIGURATION = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value
_GOAL = Pose(
    x=0.25, y=0.25, z=0.50,
    qx=0.0, qy=-0.70711, qz=0.70711, qw=0.0,
)
_OBSTACLE = SphereObstacle(center=Point(0.28, 0.28, 0.42), radius=0.03)
_CLEARANCE = 0.03

# Cuántos waypoints de la trayectoria "sin obstáculo conocido" se ejecutan
# antes de que el pseudo-perceptor lo "detecte" -- 5 de 20, deliberadamente
# a mitad de camino, ni al principio ni al final.
_DETECT_AFTER_WAYPOINT = 5
_WAYPOINT_PAUSE_SECONDS = 0.3


def main() -> None:
    ensure_coppeliasim_running(port=_ZMQ_PORT, settings_suffix=f"_perception_replan_demo_{_ZMQ_PORT}")

    # El pseudo-perceptor NO forma parte de la construcción de la escena --
    # empieza vacío a propósito (nada "sabe" del obstáculo todavía), y se
    # construye aparte de build_cr5_scene (que solo conoce el robot y su
    # postura inicial) precisamente porque su ciclo de vida es
    # independiente (ver docstring del módulo).
    perceptor = PseudoPerceptionAdapter()
    robot = build_cr5_scene(
        port=_ZMQ_PORT,
        initial_configuration=_INITIAL_CONFIGURATION,
        scene=perceptor.get_scene(),
    )
    robot.mark_goal(_GOAL)

    kinematics = PoeKinematicsAdapter()
    planner = WholeBodyObstacleAvoidingPlanningAdapter(
        kinematics, clearance=_CLEARANCE, max_detour_attempts=20
    )

    trajectory = planner.compute_trajectory(_GOAL, _INITIAL_CONFIGURATION, perceptor.get_scene())
    print(
        f"Trayectoria inicial: {len(trajectory.waypoints)} waypoints "
        f"(escena vacía -- el pseudo-perceptor todavía no ha detectado nada)"
    )

    for index, waypoint in enumerate(trajectory.waypoints):
        robot.set_joints(waypoint)
        time.sleep(_WAYPOINT_PAUSE_SECONDS)
        if index == _DETECT_AFTER_WAYPOINT:
            print(
                f"\n>>> Pseudo-perceptor: obstáculo detectado tras el "
                f"waypoint {index} <<<"
            )
            perceptor.report_obstacle("obstaculo_detectado", _OBSTACLE)
            robot.mark_obstacle(_OBSTACLE)

            current_configuration = robot.get_current_configuration()
            trajectory = planner.compute_trajectory(
                _GOAL, current_configuration, perceptor.get_scene()
            )
            print(
                f"Replanificado desde la posición actual: "
                f"{len(trajectory.waypoints)} waypoints nuevos "
                f"(evitando {len(perceptor.get_scene().obstacles)} obstáculo(s))\n"
            )
            # trajectory.waypoints[0] es la propia current_configuration --
            # el bucle for de arriba ya la envió (es la de este mismo
            # waypoint), así que se continúa desde el siguiente índice de
            # ESTA lista nueva, no desde el principio.
            for new_waypoint in trajectory.waypoints[1:]:
                robot.set_joints(new_waypoint)
                time.sleep(_WAYPOINT_PAUSE_SECONDS)
            break

    print(
        "Listo. En CoppeliaSim: el primer tramo del rastro azul va recto; "
        "a partir del punto donde apareció la esfera naranja, se desvía."
    )


if __name__ == "__main__":
    main()
