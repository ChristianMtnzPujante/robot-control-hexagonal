"""El 'Nodo Robot': adaptador de entrada/salida ROS2 alrededor de un
RobotControllerPort. No decide nada — solo traduce mensajes ROS2 <-> dominio
y delega en el adaptador concreto (CoppeliaSim, CR5 real...) elegido por
parámetro, para que la MISMA imagen ejecutable sirva para cualquiera de los
dos, sin recompilar nada.
"""

from __future__ import annotations

from rclpy.node import Node
from ros2_kit import run_node, to_joint_configuration, to_joint_state_msg
from sensor_msgs.msg import JointState

from .adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter
from .adapters.cr5_real_adapter import Cr5RealRobotAdapter


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_node")

        self.declare_parameter("robot_target", "simulado")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("state_publish_period_seconds", 0.05)

        target = self.get_parameter("robot_target").value
        joint_names = list(self.get_parameter("joint_names").value)

        self._robot_controller = self._build_adapter(target, joint_names)

        self._command_sub = self.create_subscription(
            JointState, "joint_command", self._on_joint_command, 10
        )
        self._state_pub = self.create_publisher(JointState, "joint_states", 10)

        period = float(self.get_parameter("state_publish_period_seconds").value)
        self._timer = self.create_timer(period, self._publish_state)

        self.get_logger().info(
            f'robot_node listo, target="{target}", joints={joint_names}'
        )

    def _build_adapter(self, target: str, joint_names):
        if target == "simulado":
            return CoppeliaSimRobotAdapter(joint_names)
        if target == "real":
            return Cr5RealRobotAdapter(host="192.168.1.100", port=29999)
        raise ValueError(f'robot_target desconocido: "{target}"')

    def _on_joint_command(self, msg: JointState) -> None:
        configuration = to_joint_configuration(msg)
        self._robot_controller.set_joints(configuration)

    def _publish_state(self) -> None:
        configuration = self._robot_controller.get_current_configuration()
        self._state_pub.publish(to_joint_state_msg(configuration))


def main(args=None):
    run_node(RobotNode, args=args)


if __name__ == "__main__":
    main()
