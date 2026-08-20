"""Descriptor de robot: los parámetros geométricos crudos de un manipulador
serie concreto -- la única fuente de la que PoE y (cuando se implemente) GA
derivan sus propios twists/bivectores, cada uno en su álgebra. Ver
docs/algebra_geometrica_conforme.md §4: son los mismos números interpretados
de dos formas distintas, así que aquí se guardan crudos (origen + eje por
articulación en configuración home), no ya convertidos a twists.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Literal, Tuple

from .either import Either, left, right
from .errors import InvalidRobotDescriptionError

JointType = Literal["revolute", "continuous", "prismatic"]


@dataclass(frozen=True)
class JointDescription:
    """Una articulación móvil, en configuración home, con el origen ya
    plegado respecto de la articulación móvil anterior -- cualquier
    <joint type="fixed"> intermedio del URDF ya está compuesto en este
    origen antes de llegar aquí (ver urdf_kit.parser)."""

    name: str
    joint_type: JointType
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class RobotDescription:
    """Descriptor completo de un manipulador serie: la cadena de
    articulaciones móviles entre `base_link` y `tip_link`."""

    joints: Tuple[JointDescription, ...]
    base_link: str
    tip_link: str

    @staticmethod
    def create(
        joints: List[JointDescription], base_link: str, tip_link: str
    ) -> Either[InvalidRobotDescriptionError, "RobotDescription"]:
        if not joints:
            return left(
                InvalidRobotDescriptionError(
                    "Un RobotDescription necesita al menos una articulación"
                )
            )
        if not base_link or not base_link.strip():
            return left(
                InvalidRobotDescriptionError("base_link no puede estar vacío")
            )
        if not tip_link or not tip_link.strip():
            return left(
                InvalidRobotDescriptionError("tip_link no puede estar vacío")
            )
        names = [joint.name for joint in joints]
        if any(not name or not name.strip() for name in names):
            return left(
                InvalidRobotDescriptionError(
                    "El nombre de una articulación no puede estar vacío"
                )
            )
        if len(set(names)) != len(names):
            return left(
                InvalidRobotDescriptionError(
                    f"Nombres de articulación duplicados: {names}"
                )
            )
        for joint in joints:
            axis_norm = math.sqrt(sum(component**2 for component in joint.axis))
            if axis_norm < 1e-9:
                return left(
                    InvalidRobotDescriptionError(
                        f'El eje de la articulación "{joint.name}" no puede '
                        "ser el vector nulo"
                    )
                )
        return right(RobotDescription(tuple(joints), base_link, tip_link))

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    @property
    def degrees_of_freedom(self) -> int:
        return len(self.joints)
