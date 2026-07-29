"""Puertos del dominio, como typing.Protocol.

Un adaptador satisface un puerto por su forma (duck typing verificado
estáticamente por mypy), sin necesidad de heredar de ninguna clase.
Esto es lo que hace el sistema "muy adaptable": un adaptador nuevo
solo necesita implementar estos métodos, nada más.
"""

from __future__ import annotations

from typing import Protocol

from .trajectory import Trajectory
from .value_objects import JointConfiguration, Pose


class RobotControllerPort(Protocol):
    """El 'nodo robot': ejecuta comandos crudos sobre un robot,
    simulado o real. Nunca calcula nada, solo obedece y reporta.
    """

    def set_joints(self, configuration: JointConfiguration) -> None: ...

    def get_current_configuration(self) -> JointConfiguration: ...


class KinematicsPort(Protocol):
    """El 'nodo controlador': calcula una trayectoria para alcanzar
    un objetivo cartesiano. Puede implementarse vía PoE, GA (gafro),
    DH numérico, etc. — el resto del sistema no distingue cuál es.
    """

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...
