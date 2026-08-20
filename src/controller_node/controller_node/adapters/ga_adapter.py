"""Cinemática vía álgebra geométrica conforme (gafro, Löw/Abbet/Calinon).

TODO — pendiente de verdad: falta compilar gafro + pygafro para el CR5,
y revisar la interfaz exacta de gafro_ros (confirmada su existencia en el
paper, pero no su API concreta) antes de poder implementar esto.

Nota: comprobar si gafro sabe cargar un URDF genérico directamente -- si
es así, este adaptador podría no necesitar el trabajo de generalización
de robot que sí hace falta en poe_adapter.py/dh_adapter.py (ver
ROADMAP.md, Bloque 9).

`RobotDescription` (shared_kernel) ya existe -- es la fuente de la que
`poe_adapter.py` deriva sus twists, y de la que este adaptador derivará sus
propios (B_i, t_i) cuando se implemente: mismos datos crudos por
articulación, interpretados en el álgebra de gafro en vez de en se(3) (ver
docs/algebra_geometrica_conforme.md §4).
"""

from __future__ import annotations

from typing import Optional

from shared_kernel import JointConfiguration, Pose, RobotDescription, Trajectory


class GaKinematicsAdapter:
    def __init__(self, robot_description: Optional[RobotDescription] = None) -> None:
        self._robot_description = robot_description  # sin usar todavía

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        raise NotImplementedError(
            "GaKinematicsAdapter: falta compilar/integrar gafro (pygafro) "
            "para el CR5."
        )
