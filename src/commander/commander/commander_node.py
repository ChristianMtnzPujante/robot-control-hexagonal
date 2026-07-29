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
    ) -> ControlSession:
        namespace = f"/session_{name}"
        session = ControlSession(
            namespace=namespace,
            robot_target=robot_target,
            controller_strategy=controller_strategy,
            joint_names=joint_names,
        )
        session.start()
        self._sessions[name] = session

        self._goal_publishers[name] = self.create_publisher(
            PoseMsg, f"{namespace}/goal", 10
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
        msg = PoseMsg()
        msg.position.x = goal.x
        msg.position.y = goal.y
        msg.position.z = goal.z
        msg.orientation.x = goal.qx
        msg.orientation.y = goal.qy
        msg.orientation.z = goal.qz
        msg.orientation.w = goal.qw
        self._goal_publishers[session_name].publish(msg)

    def close_session(self, session_name: str) -> None:
        self._sessions[session_name].stop()
        del self._sessions[session_name]
        del self._goal_publishers[session_name]


def main(args=None):
    """Demo mínima: crea una sesión de prueba (naive_test + simulado),
    manda un objetivo cualquiera y se queda escuchando el feedback.
    """
    rclpy.init(args=args)
    commander = Commander()

    commander.create_session(
        name="demo",
        robot_target="simulado",
        controller_strategy="naive_test",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )

    import time

    time.sleep(2.0)  # dar tiempo a que robot_node/controller_node arranquen
    commander.send_goal("demo", Pose(x=0.3, y=0.0, z=0.5))

    try:
        rclpy.spin(commander)
    finally:
        commander.close_session("demo")
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
