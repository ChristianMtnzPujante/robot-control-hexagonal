"""Cinemática vía álgebra geométrica conforme (gafro, Löw/Abbet/Calinon).

TODO — pendiente de verdad: falta compilar gafro + pygafro para el CR5,
y revisar la interfaz exacta de gafro_ros (confirmada su existencia en el
paper, pero no su API concreta) antes de poder implementar esto.
"""

from __future__ import annotations

from shared_kernel import JointConfiguration, Pose, Trajectory


class GaKinematicsAdapter:
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        raise NotImplementedError(
            "GaKinematicsAdapter: falta compilar/integrar gafro (pygafro) "
            "para el CR5."
        )
