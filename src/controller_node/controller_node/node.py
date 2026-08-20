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
from ros2_kit import GOAL_QOS, run_node, to_joint_configuration, to_joint_state_msg, to_pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from shared_kernel import JointConfiguration, Pose

from .adapters.coppeliasim_ik_adapter import CoppeliaSimIkKinematicsAdapter
from .adapters.dh_adapter import DhKinematicsAdapter
from .adapters.ga_adapter import GaKinematicsAdapter
from .adapters.naive_test_adapter import NaiveTestKinematicsAdapter
from .adapters.poe_adapter import PoeKinematicsAdapter
from .adapters.straight_line_adapter import StraightLineKinematicsAdapter


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("strategy", "naive_test")
        self.declare_parameter("waypoint_period_seconds", 0.5)
        # Puerto ZMQ del CoppeliaSim al que conectar -- solo lo usa la
        # estrategia "coppeliasim_ik". Permite varias instancias de
        # CoppeliaSim corriendo a la vez (ver commander/two_sessions_demo.py).
        self.declare_parameter("zmq_port", 23000)

        strategy = self.get_parameter("strategy").value
        zmq_port = int(self.get_parameter("zmq_port").value)
        self._kinematics = self._build_adapter(strategy, zmq_port)

        self._latest_configuration: Optional[JointConfiguration] = None
        self._pending_waypoints: list = []
        # Objetivo recibido antes de tener el primer joint_states -- se
        # procesa en cuanto llega (ver _on_joint_states), en vez de
        # descartarse. Junto con GOAL_QOS (que evita perder el mensaje si
        # el Commander publica antes de que este nodo se haya suscrito),
        # esto elimina la condición de carrera de arranque por completo:
        # ya no importa en qué orden arranquen commander/controller_node/
        # robot_node.
        self._pending_goal: Optional[Pose] = None

        self._goal_sub = self.create_subscription(
            PoseMsg, "goal", self._on_goal, GOAL_QOS
        )
        self._state_sub = self.create_subscription(
            JointState, "joint_states", self._on_joint_states, 10
        )
        self._command_pub = self.create_publisher(JointState, "joint_command", 10)
        self._feedback_pub = self.create_publisher(String, "feedback", 10)

        period = float(self.get_parameter("waypoint_period_seconds").value)
        self._timer = self.create_timer(period, self._advance_trajectory)

        self.get_logger().info(f'controller_node listo, strategy="{strategy}"')

    def _build_adapter(self, strategy: str, zmq_port: int):
        # Todos los adaptadores se construyen sin argumentos (salvo
        # zmq_port para coppeliasim_ik) -- no hay ningún parámetro ROS2 aquí
        # para decir "qué robot" (joint_names, tip, twists...); ControlSession
        # tampoco reenvía esos datos a controller_node hoy (ver
        # control_session.py::start()). Ver ROADMAP.md, Bloque 9.
        if strategy == "poe":
            return PoeKinematicsAdapter()
        if strategy == "ga":
            return GaKinematicsAdapter()
        if strategy == "dh":
            return DhKinematicsAdapter()
        if strategy == "naive_test":
            return NaiveTestKinematicsAdapter()
        if strategy == "straight_line":
            return StraightLineKinematicsAdapter()
        if strategy == "coppeliasim_ik":
            return CoppeliaSimIkKinematicsAdapter(zmq_port=zmq_port)
        raise ValueError(f'strategy desconocida: "{strategy}"')

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_configuration = to_joint_configuration(msg)
        if self._pending_goal is not None:
            goal, self._pending_goal = self._pending_goal, None
            self._start_trajectory(goal)

    def _publish_feedback(self, status: str, **extra) -> None:
        payload = {"status": status, **extra}
        self._feedback_pub.publish(String(data=json.dumps(payload)))

    def _on_goal(self, msg: PoseMsg) -> None:
        goal = to_pose(msg)
        if self._latest_configuration is None:
            # Todavía no ha llegado joint_states -- se guarda y se procesa
            # en cuanto llegue (ver _on_joint_states), no se descarta.
            self._pending_goal = goal
            self._publish_feedback("esperando_estado_robot")
            return
        self._start_trajectory(goal)

    def _start_trajectory(self, goal: Pose) -> None:
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
