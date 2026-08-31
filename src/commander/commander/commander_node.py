"""El Comandante: crea sesiones, les manda objetivos, y escucha su feedback.

Nunca sabe que existen CoppeliaSim, PoE, gafro o el CR5 real — solo conoce
namespaces de sesión y el lenguaje común Pose -> feedback.
"""

from __future__ import annotations

import json
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Pose as PoseMsg
from rclpy.node import Node
from ros2_kit import GOAL_QOS, shutdown_node, to_pose_msg
from std_msgs.msg import String

from shared_kernel import Pose

from .control_session import ControlSession


class Commander(Node):
    def __init__(self) -> None:
        super().__init__("commander")
        self._sessions: Dict[str, ControlSession] = {}
        self._goal_publishers: Dict[str, object] = {}

    def create_session(
        self,
        name: str,
        robot_target: str,
        controller_strategy: str,
        joint_names: List[str],
        waypoint_period_seconds: float = 0.5,
        tip_name: str = "",
        scene_path: str = "",
        zmq_port: int = 23000,
        urdf_path: str = "",
        base_link: str = "",
        tip_link: str = "",
    ) -> ControlSession:
        namespace = f"/session_{name}"
        session = ControlSession(
            namespace=namespace,
            robot_target=robot_target,
            controller_strategy=controller_strategy,
            joint_names=joint_names,
            waypoint_period_seconds=waypoint_period_seconds,
            tip_name=tip_name,
            scene_path=scene_path,
            zmq_port=zmq_port,
            urdf_path=urdf_path,
            base_link=base_link,
            tip_link=tip_link,
        )
        session.start()
        self._sessions[name] = session

        self._goal_publishers[name] = self.create_publisher(
            PoseMsg, f"{namespace}/goal", GOAL_QOS
        )
        self.create_subscription(
            String,
            f"{namespace}/feedback",
            lambda msg, n=name: self._on_feedback(n, msg),
            10,
        )
        return session

    def _on_feedback(self, session_name: str, msg: String) -> None:
        payload = json.loads(msg.data)
        self.get_logger().info(f"[{session_name}] {payload}")

    def send_goal(self, session_name: str, goal: Pose) -> None:
        self._goal_publishers[session_name].publish(to_pose_msg(goal))

    def close_session(self, session_name: str) -> None:
        self._sessions[session_name].stop()
        del self._sessions[session_name]
        del self._goal_publishers[session_name]


def main(args=None):
    """Demo mínima: crea una sesión (coppeliasim_ik + simulado), manda un
    objetivo cartesiano y se queda escuchando el feedback. `tip_name` debe
    coincidir con un objeto de la escena cuya posición cartesiana sirva de
    referencia del extremo del brazo: cr5_base.ttt no trae un dummy "tip"
    dedicado, así que se usa "Link6_visual" (el último eslabón visual,
    justo antes del efector final) como aproximación -- si algún día se
    añade un dummy tip/TCP propio a la escena, cambiar esto por su nombre
    (y por el mismo motivo, CoppeliaSimIkKinematicsAdapter usa ese nombre
    por defecto como tip para simIK). Esta clase de suposición ad-hoc por
    escena (qué objeto sirve de tip si la escena no lo declara) es
    justo lo que haría falta resolver por convención/manifest para cargar
    una escena arbitraria -- ver ROADMAP.md, Bloque 9. `joint_names` y
    `tip_name` sí son ya parámetros de `create_session`, así que esta
    función `main()` es solo el demo hardcodeado, no una limitación de la
    API en sí.

    El objetivo está deliberadamente cerca de la posición actual del brazo
    (unos 10cm) -- simIK resuelve por iteración local desde la
    configuración actual, y para saltos grandes puede no converger (ver
    coppeliasim_ik_adapter.py). Si cambias este Pose y la sesión falla con
    "simIK no convergió", prueba un objetivo más conservador.

    El goal se manda justo después de arrancar la sesión, sin esperar a
    que robot_node/controller_node terminen de arrancar (ver GOAL_QOS en
    ros2_kit y el buffer de `_pending_goal` en controller_node -- ninguno
    de los dos depende ya de que el orden de arranque coincida).
    """
    rclpy.init(args=args)
    commander = Commander()

    commander.create_session(
        name="demo",
        robot_target="simulado",
        controller_strategy="coppeliasim_ik",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        waypoint_period_seconds=1.5,  # pausa visible en cada waypoint
        tip_name="Link6_visual",
    )
    commander.send_goal("demo", Pose(x=-0.139, y=-0.465, z=0.096))

    try:
        rclpy.spin(commander)
    finally:
        commander.close_session("demo")
        shutdown_node(commander)


if __name__ == "__main__":
    main()
