"""El 'Nodo Robot': adaptador de entrada/salida ROS2 alrededor de un
RobotControllerPort. No decide nada — solo traduce mensajes ROS2 <-> dominio
y delega en el adaptador concreto (CoppeliaSim, CR5 real...) elegido por
parámetro, para que la MISMA imagen ejecutable sirva para cualquiera de los
dos, sin recompilar nada.
"""

from __future__ import annotations

from geometry_msgs.msg import Pose as PoseMsg
from rclpy.node import Node
from ros2_kit import (
    apply_node_config,
    load_node_config,
    package_config_path,
    run_node,
    to_joint_configuration,
    to_joint_state_msg,
    to_pose,
)
from sensor_msgs.msg import JointState

from .adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter
from .adapters.cr5_real_adapter import Cr5RealRobotAdapter

_CONFIG_PATH = package_config_path("robot_node", "robot_node.yaml")


class RobotNode(Node):
    def __init__(self) -> None:
        # 1º: leer el YAML (no toca rclpy todavía) -- hace falta el nombre
        # del propio archivo para poder construir el Node.
        config = load_node_config(_CONFIG_PATH)
        super().__init__(config.node_name)
        # 2º: ya existe self -- ahora sí se puede declarar parámetros y
        # crear publishers/subscriptions/timers (ver ros2_kit/node_config.py).
        self._topic_publishers = apply_node_config(self, config)

        # "joint_names" llega YA resuelto (literal, o derivado de un URDF
        # por quien lanzó este proceso -- ver ControlSession._resolve_joint_names
        # en commander/control_session.py). robot_node no sabe ni necesita
        # saber de dónde salió -- solo lo lee como cualquier otro parámetro.
        target = self.get_parameter("robot_target").value
        joint_names = list(self.get_parameter("joint_names").value)
        tip_name = self.get_parameter("tip_name").value or None
        scene_path = self.get_parameter("scene_path").value or None
        zmq_port = int(self.get_parameter("zmq_port").value)

        self._robot_controller = self._build_adapter(
            target, joint_names, tip_name, scene_path, zmq_port
        )

        self.get_logger().info(
            f'robot_node listo, target="{target}", joints={joint_names}'
        )

    def _build_adapter(self, target: str, joint_names, tip_name, scene_path, zmq_port):
        if target == "simulado":
            return CoppeliaSimRobotAdapter(
                joint_names, tip_name=tip_name, scene_path=scene_path, zmq_port=zmq_port
            )
        if target == "real":
            # host/port hardcodeados y Cr5RealRobotAdapter es, por diseño,
            # específico del CR5 -- añadir un segundo robot físico hoy es
            # una rama elif nueva a mano, no configuración. Ver ROADMAP.md,
            # Bloque 9.
            return Cr5RealRobotAdapter(host="192.168.1.100", port=29999)
        raise ValueError(f'robot_target desconocido: "{target}"')

    def _on_joint_command(self, msg: JointState) -> None:
        configuration = to_joint_configuration(msg)
        self._robot_controller.set_joints(configuration)

    def _on_goal(self, msg: PoseMsg) -> None:
        # mark_goal es decoración opcional (solo CoppeliaSimRobotAdapter la
        # ofrece) -- no forma parte de RobotControllerPort.
        mark_goal = getattr(self._robot_controller, "mark_goal", None)
        if mark_goal is not None:
            mark_goal(to_pose(msg))

    def _publish_state(self) -> None:
        configuration = self._robot_controller.get_current_configuration()
        self._topic_publishers["joint_states"].publish(to_joint_state_msg(configuration))


def main(args=None):
    run_node(RobotNode, args=args)


if __name__ == "__main__":
    main()
