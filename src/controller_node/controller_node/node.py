"""El 'Nodo Controlador': adaptador de entrada/salida ROS2 alrededor de un
KinematicsPort. Recibe un objetivo cartesiano, calcula la trayectoria con
la estrategia elegida (PoE/GA/DH/prueba) y va publicando los waypoints al
robot_node, reportando progreso al Commander por 'feedback'.
"""

from __future__ import annotations

import json
from typing import Optional

from geometry_msgs.msg import Pose as PoseMsg
from rclpy.node import Node
from ros2_kit import run_node, to_joint_configuration, to_joint_state_msg, to_pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from shared_kernel import JointConfiguration

from .adapters.dh_adapter import DhKinematicsAdapter
from .adapters.ga_adapter import GaKinematicsAdapter
from .adapters.naive_test_adapter import NaiveTestKinematicsAdapter
from .adapters.poe_adapter import PoeKinematicsAdapter


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("strategy", "naive_test")
        self.declare_parameter("waypoint_period_seconds", 0.5)

        strategy = self.get_parameter("strategy").value
        self._kinematics = self._build_adapter(strategy)

        self._latest_configuration: Optional[JointConfiguration] = None
        self._pending_waypoints: list = []

        self._goal_sub = self.create_subscription(
            PoseMsg, "goal", self._on_goal, 10
        )
        self._state_sub = self.create_subscription(
            JointState, "joint_states", self._on_joint_states, 10
        )
        self._command_pub = self.create_publisher(JointState, "joint_command", 10)
        self._feedback_pub = self.create_publisher(String, "feedback", 10)

        period = float(self.get_parameter("waypoint_period_seconds").value)
        self._timer = self.create_timer(period, self._advance_trajectory)

        self.get_logger().info(f'controller_node listo, strategy="{strategy}"')

    def _build_adapter(self, strategy: str):
        if strategy == "poe":
            return PoeKinematicsAdapter()
        if strategy == "ga":
            return GaKinematicsAdapter()
        if strategy == "dh":
            return DhKinematicsAdapter()
        if strategy == "naive_test":
            return NaiveTestKinematicsAdapter()
        raise ValueError(f'strategy desconocida: "{strategy}"')

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_configuration = to_joint_configuration(msg)

    def _publish_feedback(self, status: str, **extra) -> None:
        payload = {"status": status, **extra}
        self._feedback_pub.publish(String(data=json.dumps(payload)))

    def _on_goal(self, msg: PoseMsg) -> None:
        if self._latest_configuration is None:
            self._publish_feedback("error", reason="sin estado del robot todavía")
            return

        goal = to_pose(msg)
        self._publish_feedback("calculando")
        trajectory = self._kinematics.compute_trajectory(
            goal, self._latest_configuration
        )
        self._pending_waypoints = list(trajectory.waypoints)
        self._publish_feedback("trayectoria_calculada", waypoints=len(self._pending_waypoints))

    def _advance_trajectory(self) -> None:
        if not self._pending_waypoints:
            return
        waypoint = self._pending_waypoints.pop(0)
        self._command_pub.publish(to_joint_state_msg(waypoint))
        self._publish_feedback(
            "waypoint_enviado", restantes=len(self._pending_waypoints)
        )
        if not self._pending_waypoints:
            self._publish_feedback("completado")


def main(args=None):
    run_node(ControllerNode, args=args)


if __name__ == "__main__":
    main()
