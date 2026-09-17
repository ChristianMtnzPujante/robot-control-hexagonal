"""Cinemática numérica clásica sobre parámetros Denavit-Hartenberg
(Newton-Raphson/Levenberg-Marquardt sobre el Jacobiano geométrico).

TODO — pendiente de verdad: falta extraer la tabla DH del CR5 (o generarla
con el add-on "Denavit Hartenberg extractor.lua" que ya viene instalado en
CoppeliaSim) e implementar el bucle iterativo tal como se dedujo a mano
en las sesiones de teoría (residuo, Jacobiano por columnas, actualizar).

Nota: cuando se extraiga, la tabla quedará hardcodeada para el CR5 igual
que `_JOINT_ORIGINS` en `poe_adapter.py` -- generalizar a otro robot es
trabajo aparte, ver ROADMAP.md, Bloque 9.
"""

from __future__ import annotations

from typing import List

from shared_kernel import JointConfiguration, Pose, Trajectory

_NOT_IMPLEMENTED = (
    "DhKinematicsAdapter: falta implementar el bucle numérico clásico "
    "(residuo + Jacobiano + Newton-Raphson)."
)


class DhKinematicsAdapter:
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        raise NotImplementedError(_NOT_IMPLEMENTED)

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        # Parte formal de KinematicsPort desde el 08/09 (ver shared_kernel/
        # ports.py). Cinemática directa vía DH es en realidad el paso MÁS
        # simple de este adaptador (multiplicar las matrices homogéneas de
        # la tabla, sin iterar) -- se puede implementar antes que la IK
        # completa si hace falta, pero sigue pendiente de la tabla DH real
        # del CR5 (ver docstring del módulo).
        raise NotImplementedError(_NOT_IMPLEMENTED)

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]:
        raise NotImplementedError(_NOT_IMPLEMENTED)
