"""Primitivas geométricas puras, compartidas entre el contexto de
ejecución/cinemática (`shared_kernel`) y el futuro contexto de percepción
(`perception_node`, pendiente -- ver ROADMAP.md, Bloque 3).

Representación provisional: coordenadas cartesianas planas, no álgebra
geométrica conforme (CGA) real todavía (ROADMAP.md, Bloque 1). Cuando esa
investigación aterrice, estas clases podrán reimplementarse por debajo con
multivectores de verdad sin que su forma pública tenga que cambiar.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """Un objetivo cartesiano: posición + orientación (cuaternión)."""

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass(frozen=True)
class Point:
    """Un punto en el espacio, sin orientación: centro de un obstáculo,
    punto de referencia de un plano, posición de un objeto detectado...
    """

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Plane:
    """Un plano definido por un punto de referencia y su normal (ambos
    `Point`; `normal` es un vector, no una posición).
    """

    point: Point
    normal: Point


@dataclass(frozen=True)
class SphereObstacle:
    """Una región a evitar, aproximada como una esfera -- la primitiva de
    evitación más simple posible, suficiente para un CHOMP/RRT mínimo
    (ROADMAP.md, Bloque 4). Un objeto real puede aproximarse con una o
    varias de estas.
    """

    center: Point
    radius: float

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError(
                f"El radio de un obstáculo debe ser positivo, recibido: {self.radius}"
            )
