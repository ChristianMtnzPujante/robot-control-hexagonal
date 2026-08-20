"""Puertos del dominio, como typing.Protocol.

Un adaptador satisface un puerto por su forma (duck typing verificado
estáticamente por mypy), sin necesidad de heredar de ninguna clase.
Esto es lo que hace el sistema "muy adaptable": un adaptador nuevo
solo necesita implementar estos métodos, nada más.
"""

from __future__ import annotations

from typing import Protocol

from geometry_kernel import Pose, Scene

from .trajectory import Trajectory
from .value_objects import JointConfiguration


class RobotControllerPort(Protocol):
    """El 'nodo robot': ejecuta comandos crudos sobre un robot,
    simulado o real. Nunca calcula nada, solo obedece y reporta.
    """

    def set_joints(self, configuration: JointConfiguration) -> None: ...

    def get_current_configuration(self) -> JointConfiguration: ...


class KinematicsPort(Protocol):
    """Cinemática inversa pura: de un objetivo cartesiano a una trayectoria
    alcanzable, sin conocer la escena ni evitar nada. Puede implementarse
    vía PoE, GA (gafro), DH numérico, etc. La evitación de obstáculos NO es
    responsabilidad de este puerto — ver `PlanningPort`.
    """

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...


class PlanningPort(Protocol):
    """Planificación consciente de la escena: como `KinematicsPort`, pero
    recibe también la `Scene` (obstáculos, planos) y debe evitarlos.
    Adaptadores previstos: CHOMP, RRT — pendientes de implementar (ver
    ROADMAP.md, Bloque 4). Puede apoyarse en un `KinematicsPort` para
    resolver IK punto a punto, o resolver todo en espacio de
    articulaciones; esta interfaz no lo impone.
    """

    def compute_trajectory(
        self,
        goal: Pose,
        current_configuration: JointConfiguration,
        scene: Scene,
    ) -> Trajectory: ...


class PlannerSelectionPort(Protocol):
    """Elige qué estrategia de planificación usar según el estado de la
    escena (p. ej. features derivadas de `Scene`, al estilo HyperPlan).
    Devuelve el identificador de estrategia que ya consume
    `controller_node._build_adapter` ("chomp", "rrt"...). Pendiente de
    implementar (ver ROADMAP.md, Bloque 5) — de momento ningún adaptador
    lo satisface.
    """

    def select(self, scene: Scene) -> str: ...
