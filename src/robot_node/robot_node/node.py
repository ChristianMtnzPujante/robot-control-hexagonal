"""El 'Nodo Robot': adaptador de entrada/salida ROS2 alrededor de un
RobotConnectorPort. No decide nada — solo traduce mensajes ROS2 <-> dominio
y delega en el adaptador concreto (CoppeliaSim, CR5 real...) elegido por
parámetro, para que la MISMA imagen ejecutable sirva para cualquiera de los
dos, sin recompilar nada.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Sequence

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
from shared_kernel import RobotConnectorError, RobotConnectorPort

from .adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter
from .adapters.cr5_real_adapter import Cr5RealRobotAdapter

_CONFIG_PATH = package_config_path("robot_node", "robot_node.yaml")

# Registro robot_target -> factoría, en vez de un if/elif en _build_adapter:
# añadir un robot nuevo es añadir una entrada aquí, no una rama nueva (ver
# ROADMAP.md, Bloque 9). Todas las factorías reciben las mismas SIETE
# cosas que hoy expone robot_node por parámetro ROS2 -- la que no las
# necesite simplemente las ignora (Cr5RealRobotAdapter usa joint_names/
# cr5_host/cr5_movj_cp/cr5_joint_limits_degrees, pero no
# tip_name/scene_path/zmq_port). Ese es justo el límite del patrón: quita
# el if/elif, pero NO resuelve que un robot real pueda necesitar datos de
# conexión/ajuste que estas siete no cubran.
_TARGETS: Dict[
    str,
    Callable[
        [List[str], Optional[str], Optional[str], int, str, int, Sequence[float]],
        RobotConnectorPort,
    ],
] = {
    "simulado": lambda joint_names, tip_name, scene_path, zmq_port, cr5_host, cr5_movj_cp, cr5_joint_limits_degrees: (
        CoppeliaSimRobotAdapter(
            joint_names, tip_name=tip_name, scene_path=scene_path, zmq_port=zmq_port
        )
    ),
    "real": lambda joint_names, tip_name, scene_path, zmq_port, cr5_host, cr5_movj_cp, cr5_joint_limits_degrees: (
        Cr5RealRobotAdapter(
            host=cr5_host,
            joint_names=joint_names,
            movj_cp=cr5_movj_cp,
            joint_limits_degrees=list(cr5_joint_limits_degrees),
        )
    ),
}


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
        cr5_host = self.get_parameter("cr5_host").value
        cr5_movj_cp = int(self.get_parameter("cr5_movj_cp").value)
        cr5_joint_limits_degrees = list(
            self.get_parameter("cr5_joint_limits_degrees").value
        )

        self._robot_controller = self._build_adapter(
            target,
            joint_names,
            tip_name,
            scene_path,
            zmq_port,
            cr5_host,
            cr5_movj_cp,
            cr5_joint_limits_degrees,
        )

        self.get_logger().info(
            f'robot_node listo, target="{target}", joints={joint_names}'
        )

    def _build_adapter(
        self,
        target: str,
        joint_names,
        tip_name,
        scene_path,
        zmq_port,
        cr5_host,
        cr5_movj_cp,
        cr5_joint_limits_degrees,
    ) -> RobotConnectorPort:
        factory = _TARGETS.get(target)
        if factory is None:
            raise ValueError(f'robot_target desconocido: "{target}"')
        return factory(
            joint_names,
            tip_name,
            scene_path,
            zmq_port,
            cr5_host,
            cr5_movj_cp,
            cr5_joint_limits_degrees,
        )

    def _on_joint_command(self, msg: JointState) -> None:
        configuration = to_joint_configuration(msg)
        try:
            self._robot_controller.set_joints(configuration)
        except RobotConnectorError as error:
            # CORREGIDO 07/09 (Bloque 0 #116): antes esto no se capturaba
            # y un solo fallo de protocolo (visto en vivo: reset de
            # conexión a mitad de una trayectoria real) tumbaba el nodo
            # entero -- ver la sesión de #110. Cr5CommandSocket ya
            # reconecta y reintenta sola una vez (ver _cr5_protocol.py)
            # antes de llegar aquí, así que si esto se ve, ya se agotó
            # ese margen: loggear y seguir vivo es mejor que morir, pero
            # OJO -- este waypoint concreto se pierde, no se reintenta
            # desde aquí (ver nota de la tarea sobre política de
            # reintento/degradación, todavía sin decidir del todo).
            self.get_logger().error(f"Fallo mandando joint_command al robot: {error}")

    def _on_goal(self, msg: PoseMsg) -> None:
        # mark_goal es decoración opcional (solo CoppeliaSimRobotAdapter la
        # ofrece) -- no forma parte de RobotConnectorPort.
        mark_goal = getattr(self._robot_controller, "mark_goal", None)
        if mark_goal is not None:
            mark_goal(to_pose(msg))

    def _publish_state(self) -> None:
        # Mismo motivo que _on_joint_command (Bloque 0 #116): esto corre
        # en un timer rápido (por defecto cada 50ms, ver
        # state_publish_period_seconds) contra el mismo canal de red que
        # puede fallar -- sin este try/except, un hipo en la LECTURA de
        # estado tumbaría el nodo tan fácilmente como uno en el envío de
        # un comando. Se salta esta publicación (no hay nada bueno que
        # publicar) y se reintenta solo en el siguiente tick del timer.
        try:
            configuration = self._robot_controller.get_current_configuration()
        except RobotConnectorError as error:
            self.get_logger().error(f"Fallo leyendo el estado del robot: {error}")
            return
        self._topic_publishers["joint_states"].publish(to_joint_state_msg(configuration))

    def destroy_node(self) -> None:
        # CORREGIDO 07/09 (Bloque 0 #116): robot_node no cerraba/des-
        # energizaba el adaptador en NINGÚN camino de cierre -- ni normal
        # (ControlSession.stop() manda SIGTERM; rclpy.init() ya instala un
        # manejador para SIGINT y SIGTERM por defecto, así que rclpy.spin()
        # retorna limpio y esto se ejecuta vía el try/finally de
        # ros2_kit.run_node) ni por un crash (mismo try/finally). Un fallo
        # cerrando no debe impedir el resto del cierre del nodo -- se
        # loggea, no se relanza.
        try:
            self._robot_controller.close()
        except RobotConnectorError as error:
            self.get_logger().error(
                f"Fallo cerrando el adaptador del robot (puede haber quedado "
                f"energizado): {error}"
            )
        return super().destroy_node()


def main(args=None):
    run_node(RobotNode, args=args)


if __name__ == "__main__":
    main()
