"""El 'Nodo Controlador': adaptador de entrada/salida ROS2 alrededor de un
KinematicsPort. Recibe un objetivo cartesiano, calcula la trayectoria con
la estrategia elegida (PoE/GA/DH/prueba) y va publicando los waypoints al
robot_node, reportando progreso al Commander por 'feedback'.

La estrategia no queda fija al arrancar: el topic 'set_strategy' (ver
_on_set_strategy) permite cambiarla en caliente desde cualquier otro nodo
-- incluido un futuro supervisor en otra máquina (ROS2/DDS no distingue
local de remoto), sin relanzar la sesión (ROADMAP.md, Bloque 7). Solo
afecta al PRÓXIMO objetivo: una trayectoria ya en curso (_pending_waypoints)
sigue drenándose con la estrategia con la que se calculó -- cancelarla a
medio camino es "replanificación reactiva" de verdad (Bloque 4), no algo
que este mecanismo resuelva todavía.
"""

from __future__ import annotations

import json
from typing import Optional

from geometry_msgs.msg import Pose as PoseMsg
from rclpy.node import Node
from ros2_kit import (
    GOAL_QOS,
    STRATEGY_QOS,
    run_node,
    to_joint_configuration,
    to_joint_state_msg,
    to_pose,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from shared_kernel import JointConfiguration, Pose, RobotDescription
from urdf_kit import parse_urdf_file

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
        # Ruta a un .urdf real (más base_link/tip_link, la cadena serie a
        # extraer de él) para construir un RobotDescription genérico -- solo
        # lo usan las estrategias "poe"/"ga". Vacío conserva el CR5
        # hardcodeado que cada adaptador trae como default (ver
        # ROADMAP.md, Bloque 9).
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_link", "")
        self.declare_parameter("tip_link", "")

        # zmq_port/urdf_path/base_link/tip_link no cambian a media sesión
        # (son del robot/escena, no de la estrategia) -- se guardan para que
        # _on_set_strategy pueda reconstruir el adaptador más tarde sin
        # tener que volver a declararlos ni pedirlos por el mensaje.
        self._zmq_port = int(self.get_parameter("zmq_port").value)
        self._urdf_path = self.get_parameter("urdf_path").value
        self._base_link = self.get_parameter("base_link").value
        self._tip_link = self.get_parameter("tip_link").value

        strategy = self.get_parameter("strategy").value
        self._strategy = strategy
        self._kinematics = self._build_adapter(strategy)

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
        # Canal de control para que un supervisor (Bloque 7, incluso en otra
        # máquina -- ROS2/DDS no distingue local de remoto) pueda cambiar la
        # estrategia de cinemática en caliente, sin relanzar la sesión.
        # best-effort a propósito (ver STRATEGY_QOS): perder un mensaje no es
        # crítico, la sesión sigue con la estrategia anterior.
        self._strategy_sub = self.create_subscription(
            String, "set_strategy", self._on_set_strategy, STRATEGY_QOS
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
        # Lee zmq_port/urdf_path/base_link/tip_link de self -- no cambian
        # entre llamadas, así que _on_set_strategy puede llamar a esto de
        # nuevo pasando solo la estrategia nueva (ver ROADMAP.md, Bloque 9).
        robot_description = self._load_robot_description()
        if strategy == "poe":
            if robot_description is None:
                return PoeKinematicsAdapter()
            return PoeKinematicsAdapter(robot_description=robot_description)
        if strategy == "ga":
            return GaKinematicsAdapter(robot_description=robot_description)
        if strategy == "dh":
            return DhKinematicsAdapter()
        if strategy == "naive_test":
            return NaiveTestKinematicsAdapter()
        if strategy == "straight_line":
            return StraightLineKinematicsAdapter()
        if strategy == "coppeliasim_ik":
            return CoppeliaSimIkKinematicsAdapter(zmq_port=self._zmq_port)
        raise ValueError(f'strategy desconocida: "{strategy}"')

    def _load_robot_description(self) -> Optional[RobotDescription]:
        # Vacío -- ningún robot cargado, cada adaptador usa su propio
        # default (hoy, el CR5 hardcodeado). No aplica a "dh"/"naive_test"/
        # "straight_line"/"coppeliasim_ik", que no leen RobotDescription.
        if not self._urdf_path:
            return None
        if not self._base_link or not self._tip_link:
            raise ValueError(
                'urdf_path requiere también "base_link" y "tip_link" (la '
                "cadena serie a extraer del URDF) -- ninguno puede estar vacío."
            )
        return parse_urdf_file(self._urdf_path, self._base_link, self._tip_link)

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_configuration = to_joint_configuration(msg)
        if self._pending_goal is not None:
            goal, self._pending_goal = self._pending_goal, None
            self._start_trajectory(goal)

    def _publish_feedback(self, status: str, **extra) -> None:
        payload = {"status": status, **extra}
        self._feedback_pub.publish(String(data=json.dumps(payload)))

    def _on_set_strategy(self, msg: String) -> None:
        new_strategy = msg.data
        try:
            new_kinematics = self._build_adapter(new_strategy)
        except Exception as error:
            # Límite del sistema: un mensaje externo puede pedir cualquier
            # cosa (estrategia desconocida, urdf_path que ya no existe...).
            # No se propaga -- la sesión sigue con la estrategia anterior,
            # que sigue siendo válida (best-effort, ver STRATEGY_QOS).
            self.get_logger().warning(
                f'set_strategy: no se pudo cambiar a "{new_strategy}": {error}'
            )
            self._publish_feedback(
                "estrategia_invalida", strategy=new_strategy, error=str(error)
            )
            return
        self._kinematics = new_kinematics
        self._strategy = new_strategy
        self.get_logger().info(f'controller_node: estrategia cambiada a "{new_strategy}"')
        self._publish_feedback("estrategia_cambiada", strategy=new_strategy)

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
