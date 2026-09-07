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


class RobotConnectorError(Exception):
    """Fallo al ejecutar un comando sobre el robot a través de un
    RobotConnectorPort -- de comunicación (red, protocolo) o del propio
    robot rechazando el comando, NUNCA un error de programación. robot_node
    la captura genéricamente (ver _on_joint_command) para no morir por un
    hipo de red -- por eso vive aquí, junto al puerto, y no dentro de un
    adaptador concreto: robot_node no debe saber que Cr5ProtocolError
    existe (rompería el 'la misma imagen sirve para cualquier robot' que
    es la razón de ser de este puerto). Cada adaptador que necesite más
    detalle puede definir su propia subclase (ver Cr5ProtocolError) y
    seguirá siendo capturable como RobotConnectorError."""


class RobotConnectorPort(Protocol):
    """El 'nodo robot': ejecuta comandos crudos sobre un robot,
    simulado o real. Nunca calcula nada, solo obedece y reporta.
    """

    def set_joints(self, configuration: JointConfiguration) -> None: ...

    def get_current_configuration(self) -> JointConfiguration: ...

    def close(self) -> None:
        """Libera lo que este adaptador tenga abierto (sockets, clientes
        de simulador...) y deja el robot en un estado seguro si aplica
        (p. ej. Cr5RealRobotAdapter des-energiza antes de cerrar). Parte
        formal del puerto desde el 07/09 (Vikunja Bloque 0 #116) -- antes
        era un método suelto solo en Cr5RealRobotAdapter, y robot_node no
        lo llamaba en ningún camino de cierre porque no formaba parte del
        contrato que robot_node conoce. Un adaptador sin nada que liberar
        (p. ej. CoppeliaSimRobotAdapter hoy) lo implementa como no-op."""
        ...


class KinematicsPort(Protocol):
    """Cinemática inversa pura: de un objetivo cartesiano a una trayectoria
    alcanzable, sin conocer la escena ni evitar nada. Puede implementarse
    vía PoE, GA (gafro), DH numérico, etc. La evitación de obstáculos NO es
    responsabilidad de este puerto — ver `PlanningPort`.
    """

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory: ...


class PerceptionPort(Protocol):
    """Lo que el sistema sabe de la escena en este instante: 'detecta plano
    X', 'lista obstáculos actuales'. Desacoplado de la implementación de
    visión (ground truth de CoppeliaSim, cámara real...) -- quien consume
    este puerto (`PlanningPort`, `PlannerSelectionPort`) nunca sabe cuál de
    las dos hay detrás. Devuelve una `Scene` completa de una vez (no
    streaming/eventos) porque hoy no hace falta más: primer consumidor real,
    ver ROADMAP.md, Bloque 3.
    """

    def get_scene(self) -> Scene: ...


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
