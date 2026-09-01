"""Primitivas geométricas puras, compartidas entre el contexto de
ejecución/cinemática (`shared_kernel`) y el futuro contexto de percepción
(`perception_node`, pendiente -- ver ROADMAP.md, Bloque 3).

Representación cartesiana -- la que usan PoE/DH, no álgebra geométrica
conforme (CGA) real. Decisión revisada (ver ROADMAP.md, Bloque 1): estas
clases NO se reimplementarán por debajo con multivectores cuando GA
aterrice -- PoE y CGA son bounded contexts distintos, cada uno con su
propio lenguaje geométrico (cartesiano vs conforme), no un modelo
compartido cuyo interior se intercambia sin más (eso sería forzar un
Shared Kernel entre dos álgebras que no tienen por qué acoplarse). En su
lugar, un futuro paquete de primitivas conformes definiría sus propios
tipos (multivectores; tabla de traducción ya en
docs/algebra_geometrica_conforme.md §2), y quien construya la escena
traduciría explícitamente entre ambos -- cada `KinematicsPort`/
`PlanningPort` consume la representación que corresponda a su álgebra.
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
