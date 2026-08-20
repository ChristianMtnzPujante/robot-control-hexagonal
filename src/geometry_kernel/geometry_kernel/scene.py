"""Scene: lo que el sistema sabe de la escena en un instante dado --
planos con nombre, obstáculos, objetos detectados.

La produce percepción (ROADMAP.md, Bloque 3, sin adaptador todavía) y la
consumen planificación (Bloque 4/5) y grounding (Bloque 6). Mientras no
exista ningún productor real, `Scene.empty()` es el punto de partida.

Inmutable a propósito (como el resto de value objects del dominio): cada
`with_*` devuelve una `Scene` nueva en vez de mutar la existente.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from .primitives import Plane, Point, SphereObstacle


@dataclass(frozen=True)
class Scene:
    planes: Dict[str, Plane] = field(default_factory=dict)
    obstacles: List[SphereObstacle] = field(default_factory=list)
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

    def with_obstacle(self, obstacle: SphereObstacle) -> "Scene":
        return Scene(
            planes=self.planes,
            obstacles=[*self.obstacles, obstacle],
            objects=self.objects,
        )

    def with_object(self, name: str, position: Point) -> "Scene":
        return Scene(
            planes=self.planes,
            obstacles=self.obstacles,
            objects={**self.objects, name: position},
        )
