"""Utilidad para construir/gestionar un puente ros1_bridge de forma
programática -- BOCETO, sin usar todavía en ningún adaptador, y sin
probar contra un ROS1 real (no hay ROS1 instalado en este entorno).

Motivación: cuando llegue el momento de conectar el CR5 real vía su
driver oficial (dobot_bringup, que es ROS1/catkin -- ver README.md),
la opción "puente" necesitará levantar y configurar un proceso
ros1_bridge. En vez de que ese futuro adaptador (p.ej.
'Cr5BridgeRobotAdapter', en robot_node/adapters/) tenga que hacerlo a
mano, esta clase encapsularía esas dos tareas -- exactamente el mismo
papel que ControlSession hace para robot_node/controller_node, pero
para el propio puente.

Una vez el puente está corriendo, el adaptador que lo usa seguiría
hablando rclpy normal y corriente contra los topics ya traducidos --
no necesitaría importar rospy en ningún momento. Por eso este paquete
en sí no depende de rospy: solo orquesta un proceso `ros1_bridge` desde
fuera (subprocess), igual que ControlSession orquesta robot_node/
controller_node sin ser ella misma un nodo ROS2.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class BridgedTopic:
    """Un topic a traducir entre ROS1 y ROS2."""

    topic: str
    ros1_type: str  # p.ej. "sensor_msgs/JointState"
    ros2_type: str  # p.ej. "sensor_msgs/msg/JointState"


@dataclass(frozen=True)
class Ros1BridgeConfig:
    topics: List[BridgedTopic] = field(default_factory=list)


class Ros1Bridge:
    """Ciclo de vida de un proceso ros1_bridge -- análogo a ControlSession,
    pero para el puente en sí, no para nuestros propios nodos.
    """

    def __init__(self, config: Ros1BridgeConfig):
        self._config = config
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        # TODO: generar la configuración de topics a partir de self._config
        # (ros1_bridge admite un YAML de topics explícitos, o el modo
        # "bridge-all-topics" más simple pero menos controlado), y lanzar
        # el proceso correspondiente. Requiere tener ROS1 (Noetic) y ROS2
        # instalados y "sourceados" a la vez en la misma máquina -- no
        # disponible en este entorno todavía, por eso queda sin implementar.
        raise NotImplementedError(
            "Ros1Bridge.start(): pendiente -- requiere una instalación de "
            "ROS1 (Noetic) conviviendo con ROS2 Humble, no disponible en "
            "este entorno. Ver el comentario de este método antes de "
            "implementarlo."
        )

    def stop(self) -> None:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()

    def __enter__(self) -> "Ros1Bridge":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()
