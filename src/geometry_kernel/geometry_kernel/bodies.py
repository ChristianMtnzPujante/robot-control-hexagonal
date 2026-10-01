"""Cuerpos sólidos de la escena: una forma, dónde está y qué papel tiene.

A diferencia de `SphereObstacle` (una región a evitar, pensada para los
planificadores) o de `Scene.objects` (puntos con nombre, hoy usado para el
objetivo del movimiento), un `Body` describe una cosa física de la escena:
una mesa, un cubo que se puede coger... Es la primera pieza del vocabulario
de objetos de la ontología del dominio (ver el vault, "Ontología del Dominio
(lenguaje CGA)"): forma + pose + papel, sin saber nada de ningún simulador.

Los planificadores siguen trabajando con esferas: `Body.bounding_sphere()`
la deriva de la forma, para no tener que describir el mismo objeto dos
veces.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple, Union

from .primitives import Point, Pose, SphereObstacle


def _require_positive(**values: float) -> None:
    for name, value in values.items():
        if value <= 0:
            raise ValueError(f"{name} debe ser > 0, no {value}")


@dataclass(frozen=True)
class Box:
    """Caja centrada en la pose del cuerpo; medidas totales en metros."""

    size_x: float
    size_y: float
    size_z: float

    def __post_init__(self) -> None:
        _require_positive(size_x=self.size_x, size_y=self.size_y, size_z=self.size_z)

    def bounding_radius(self) -> float:
        return 0.5 * math.sqrt(self.size_x**2 + self.size_y**2 + self.size_z**2)


@dataclass(frozen=True)
class Cylinder:
    """Cilindro centrado en la pose del cuerpo, con el eje en su z local."""

    radius: float
    height: float

    def __post_init__(self) -> None:
        _require_positive(radius=self.radius, height=self.height)

    def bounding_radius(self) -> float:
        return math.sqrt(self.radius**2 + (0.5 * self.height) ** 2)


@dataclass(frozen=True)
class Sphere:
    radius: float

    def __post_init__(self) -> None:
        _require_positive(radius=self.radius)

    def bounding_radius(self) -> float:
        return self.radius


Shape = Union[Box, Cylinder, Sphere]


@dataclass(frozen=True)
class Body:
    """Un cuerpo sólido de la escena.

    `pose` es la del CENTRO de la forma, en el mismo marco que el resto de la
    `Scene` (base del robot == mundo de CoppeliaSim, ver
    `coppeliasim_scene_builder`). `graspable` distingue lo que la pinza
    puede coger de lo que está fijo (una mesa, una pared). `color` es
    [r, g, b] en 0..1, solo para verlo; sin él, quien lo dibuje elige.
    """

    shape: Shape
    pose: Pose
    graspable: bool = False
    color: Optional[Tuple[float, float, float]] = None

    def bounding_sphere(self) -> SphereObstacle:
        """La esfera más pequeña centrada en el cuerpo que lo contiene
        entero, para los planificadores que solo saben de esferas. Holgada
        para formas alargadas (una mesa): para eso habrá que trocearla en
        varias."""
        center = Point(self.pose.x, self.pose.y, self.pose.z)
        return SphereObstacle(center, self.shape.bounding_radius())
