"""Value objects del dominio: el vocabulario común a todos los adaptadores.

Ninguna clase de este archivo sabe qué es ROS2, CoppeliaSim, PoE o gafro.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .either import Either, left, right
from .errors import EmptyJointConfigurationError, InvalidJointPositionError


@dataclass(frozen=True)
class JointPosition:
    """El ángulo de una única articulación en un instante dado."""

    joint_name: str
    angle_radians: float

    @staticmethod
    def create(
        joint_name: str, angle_radians: float
    ) -> Either[InvalidJointPositionError, "JointPosition"]:
        if not joint_name or not joint_name.strip():
            return left(
                InvalidJointPositionError(
                    "El nombre de la articulación no puede estar vacío"
                )
            )
        if not math.isfinite(angle_radians):
            return left(
                InvalidJointPositionError(
                    f'Ángulo inválido para "{joint_name}": {angle_radians}'
                )
            )
        return right(JointPosition(joint_name, angle_radians))


@dataclass(frozen=True)
class JointConfiguration:
    """Un waypoint: las posiciones de todas las articulaciones en un instante."""

    positions: List[JointPosition]

    @staticmethod
    def create(
        positions: List[JointPosition],
    ) -> Either[EmptyJointConfigurationError, "JointConfiguration"]:
        if not positions:
            return left(EmptyJointConfigurationError())
        return right(JointConfiguration(list(positions)))

    def angle_of(self, joint_name: str) -> float:
        for p in self.positions:
            if p.joint_name == joint_name:
                return p.angle_radians
        raise KeyError(f'No hay ninguna articulación llamada "{joint_name}"')
