"""Demo de cableado end-to-end, de punta a punta por ROS2 de verdad (no
llamadas Python directas como `perception_replan_demo.py`): escribir
"objetivo x y z" en el fichero que lee `perception_node` hace que el
brazo simulado se mueva ahí, sin tocar código.

Cadena completa:
  1. `perception_node` (proceso aparte, ver más abajo) relee su fichero
     cada `scene_publish_period_seconds` y publica la `Scene` resultante
     en `/perception/scene` (JSON en `std_msgs/String`, ver
     `ros2_kit.to_scene_msg`).
  2. Este proceso (`Commander`) escucha ese topic (`follow_perception`) y,
     en cuanto ve un objetivo NUEVO bajo `Scene.objects["objetivo"]`, lo
     reenvía como `Pose` a la sesión con `send_goal` -- el mismo camino
     que ya usa `commander_node.py:main` para su goal hardcodeado.
  3. `controller_node` de la sesión recibe el `Pose` en `<ns>/goal`
     (`_on_goal`) y calcula una trayectoria nueva desde la configuración
     ACTUAL del robot -- sin necesitar saber que el goal vino de un
     fichero y no de código (ver controller_node/node.py). Mientras no
     llega ningún goal nuevo, se queda "esperando" (sin
     `_pending_waypoints` que drenar): activo, pero comportándose como si
     ya hubiera llegado al último objetivo -- exactamente el estado que
     pedías para probar esto.

Requiere DOS terminales, además de esta:

    # terminal 1: CoppeliaSim ya abierto con cr5_base.ttt cargado y en
    # play (mismo supuesto que commander_node.py:main -- ver
    # CoppeliaSimRobotAdapter, "sin scene_path asume que la escena ya
    # está cargada y en play")

    # terminal 2: el perceptor, apuntando al fichero de ejemplo
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 run perception_node perception_node --ros-args \\
        -p file_path:=$(pwd)/src/perception_node/example_obstacles.txt \\
        -p scene_publish_period_seconds:=0.5

Uso: `ros2 run commander file_perception_goal_demo`, y luego, desde una
CUARTA terminal, añade una línea "objetivo x y z" al fichero de
`file_path` (o edita la que ya tenga) -- en el siguiente ciclo del
perceptor, el brazo se mueve.
"""

from __future__ import annotations

import rclpy
from ros2_kit import shutdown_node

from .commander_node import Commander

_SESSION_NAME = "demo"
_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def main(args=None) -> None:
    rclpy.init(args=args)
    commander = Commander()

    commander.create_session(
        name=_SESSION_NAME,
        robot_target="simulado",
        controller_strategy="coppeliasim_ik",
        joint_names=_JOINT_NAMES,
        waypoint_period_seconds=1.5,  # pausa visible en cada waypoint
        tip_name="Link6_visual",
    )
    commander.follow_perception(_SESSION_NAME)
    commander.get_logger().info(
        f'Sesión "{_SESSION_NAME}" lista, sin objetivo inicial -- '
        'escuchando /perception/scene. Escribe "objetivo x y z" en el '
        "fichero de perception_node para mover el brazo."
    )

    try:
        rclpy.spin(commander)
    finally:
        commander.close_session(_SESSION_NAME)
        shutdown_node(commander)


if __name__ == "__main__":
    main()
