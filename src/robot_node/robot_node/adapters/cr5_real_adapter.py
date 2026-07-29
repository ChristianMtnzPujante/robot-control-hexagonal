"""Adaptador de salida (driven adapter) hacia el CR5 físico.

TODO — pendiente de verdad, no un placeholder cosmético: el driver oficial
de Dobot (TCP-IP-ROS-6AXis / dobot_bringup) es ROS1 (catkin), no ROS2.
Para implementar esto de forma honesta hace falta, como mínimo, reimplementar
el protocolo TCP/IP del `tcp_socket.cpp` del fabricante (o levantar un puente
ros1_bridge) antes de que este adaptador pueda hacer algo real.

Se deja aquí, ya con la forma exacta que exige RobotControllerPort, para que
quede claro dónde exactamente hay que enchufar ese trabajo cuando se haga.
"""

from __future__ import annotations

from shared_kernel import JointConfiguration


class Cr5RealRobotAdapter:
    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        raise NotImplementedError(
            "Cr5RealRobotAdapter: falta implementar el protocolo TCP/IP real "
            "del CR5 (ver dobot_bringup/src/tcp_socket.cpp del fabricante, "
            "que es ROS1). No usar todavía en producción."
        )

    def set_joints(self, configuration: JointConfiguration) -> None:
        raise NotImplementedError

    def get_current_configuration(self) -> JointConfiguration:
        raise NotImplementedError
