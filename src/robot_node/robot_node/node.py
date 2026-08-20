"""El 'Nodo Robot': adaptador de entrada/salida ROS2 alrededor de un
RobotControllerPort. No decide nada — solo traduce mensajes ROS2 <-> dominio
y delega en el adaptador concreto (CoppeliaSim, CR5 real...) elegido por
parámetro, para que la MISMA imagen ejecutable sirva para cualquiera de los
dos, sin recompilar nada.
"""

from __future__ import annotations

from geometry_msgs.msg import Pose as PoseMsg
from rclpy.node import Node
from ros2_kit import run_node, to_joint_configuration, to_joint_state_msg, to_pose
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
        # Nombre del dummy/objeto "tip" en la escena de CoppeliaSim, solo
        # para decoración visual (dejar un trail de waypoints). Vacío
        # desactiva esa decoración. No afecta a RobotControllerPort.
        self.declare_parameter("tip_name", "")
        # Ruta a una escena .ttt de CoppeliaSim a cargar (y poner en play)
        # automáticamente al arrancar. Vacío asume que ya hay una escena
        # cargada y en play (comportamiento de siempre).
        self.declare_parameter("scene_path", "")
        # Puerto ZMQ del CoppeliaSim al que conectar -- permite tener varias
        # instancias de CoppeliaSim corriendo a la vez, cada una con su
        # propia ControlSession (ver commander/two_sessions_demo.py).
        self.declare_parameter("zmq_port", 23000)

        target = self.get_parameter("robot_target").value
        joint_names = list(self.get_parameter("joint_names").value)
        tip_name = self.get_parameter("tip_name").value or None
        scene_path = self.get_parameter("scene_path").value or None
        zmq_port = int(self.get_parameter("zmq_port").value)

        self._robot_controller = self._build_adapter(
            target, joint_names, tip_name, scene_path, zmq_port
        )

        self._command_sub = self.create_subscription(
            JointState, "joint_command", self._on_joint_command, 10
        )
        self._goal_sub = self.create_subscription(
            PoseMsg, "goal", self._on_goal, 10
        )
        self._state_pub = self.create_publisher(JointState, "joint_states", 10)

        period = float(self.get_parameter("state_publish_period_seconds").value)
        self._timer = self.create_timer(period, self._publish_state)

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
        self._state_pub.publish(to_joint_state_msg(configuration))


def main(args=None):
    run_node(RobotNode, args=args)


if __name__ == "__main__":
    main()
