"""Scene: lo que el sistema sabe de la escena en un instante dado --
planos con nombre, obstáculos con nombre, objetos detectados.

La produce percepción (ROADMAP.md, Bloque 3) y la consumen planificación
(Bloque 4/5) y grounding (Bloque 6). Mientras no exista ningún productor
real, `Scene.empty()` es el punto de partida.

Inmutable a propósito (como el resto de value objects del dominio): cada
`with_*`/`merge` devuelve una `Scene` nueva en vez de mutar la existente.

`obstacles` es `Dict[str, SphereObstacle]`, igual que `planes`/`objects` --
no una lista (decisión tomada al diseñar `perception_node`, ver
ROADMAP.md Bloque 3): un obstáculo reportado dos veces con el mismo nombre
se actualiza/sobrescribe en vez de duplicarse, y con nombre estable un
productor externo (fichero, ground truth de CoppeliaSim...) puede releer
su fuente entera en cada ciclo y reconstruir el dict sin tener que llevar
la cuenta de qué era "nuevo".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

from .primitives import Plane, Point, SphereObstacle


@dataclass(frozen=True)
class Scene:
    planes: Dict[str, Plane] = field(default_factory=dict)
    obstacles: Dict[str, SphereObstacle] = field(default_factory=dict)
    objects: Dict[str, Point] = field(default_factory=dict)

    @staticmethod
    def empty() -> "Scene":
        return Scene()

    def with_plane(self, name: str, plane: Plane) -> "Scene":
        return Scene(
            planes={**self.planes, name: plane},
            obstacles=self.obstacles,
            objects=self.objects,
        )

    def with_obstacle(self, name: str, obstacle: SphereObstacle) -> "Scene":
        return Scene(
            planes=self.planes,
            obstacles={**self.obstacles, name: obstacle},
            objects=self.objects,
        )

    def with_object(self, name: str, position: Point) -> "Scene":
        return Scene(
            planes=self.planes,
            obstacles=self.obstacles,
            objects={**self.objects, name: position},
        )

    def merge(self, other: "Scene") -> "Scene":
        """Combina esta `Scene` con `other`, clave a clave -- pensado para
        quien ensambla varias piezas parciales (p. ej. un obstáculo
        reportado por un perceptor, un plano reportado por otro) en una
        única `Scene` completa (ver ROADMAP.md Bloque 3: `Commander` como
        ensamblador). En un choque de claves gana `other`, mismo criterio
        de "sobrescribe por clave" que ya usan `with_plane`/`with_object`/
        `with_obstacle` -- `merge` no es más que aplicar ese mismo criterio
        a dos `Scene` de una vez en lugar de a un solo elemento."""
        return Scene(
            planes={**self.planes, **other.planes},
            obstacles={**self.obstacles, **other.obstacles},
            objects={**self.objects, **other.objects},
        )
