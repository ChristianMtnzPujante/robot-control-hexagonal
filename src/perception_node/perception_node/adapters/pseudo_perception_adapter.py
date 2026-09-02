"""Segundo adaptador de `PerceptionPort` (shared_kernel): a diferencia de
`StaticPerceptionAdapter` (una `Scene` fija desde construcción), este
permite "inyectar" eventos con el tiempo -- un nuevo obstáculo
"detectado", un nuevo objetivo a seguir -- sin cámara ni visión real
todavía. Percepción puramente programática: primera fase del Bloque 3 del
ROADMAP ("Pseudo-perceptor"), pensada para poder probar replanificación
de verdad (Bloque 4: "Replanificación local cuando cambia el campo de
obstáculos") sin depender de que exista percepción real.

Ciclo de vida (ver ROADMAP.md Bloque 3 y Vikunja #89, spike ya resuelto):
INDEPENDIENTE del de cualquier `ControlSession` -- quien lo use (hoy, un
demo; en el futuro, un nodo de percepción propio) lo construye y lo
alimenta con `report_obstacle`/`report_object`; `Commander` (o quien
construya la sesión) se limita a ESCUCHAR su `get_scene()`, nunca lo crea
ni lo posee de la forma en que posee una `ControlSession`.

Auto-descripción (ver ROADMAP Bloque 3/6 y Vikunja #88): `description`
declara qué tipo de información reporta este adaptador, al estilo del
schema de una tool de MCP -- nadie lo consume todavía (no hay LLM en el
bucle), pero nace con esta forma para no tener que añadirla más tarde.
"""

from __future__ import annotations

from typing import Optional

from shared_kernel import Point, Scene, SphereObstacle


class PseudoPerceptionAdapter:
    description = {
        "name": "pseudo_perceptor",
        "description": (
            "Percepción programática: obstáculos y objetivos inyectados a "
            "mano, sin cámara ni visión real -- primera fase de "
            "PerceptionPort dinámico."
        ),
        "reports": {
            "obstacles": (
                "SphereObstacle -- centro (x,y,z) y radio, en el marco "
                "cartesiano relativo a base_link (mismo que exige "
                "KinematicsPort)."
            ),
            "objects": (
                "Point con nombre -- posición (x,y,z) de un objeto/objetivo "
                "detectado, mismo marco."
            ),
        },
    }

    def __init__(self, initial_scene: Optional[Scene] = None) -> None:
        # Empieza vacía si no se da nada -- mismo punto de partida que
        # Scene.empty() en StaticPerceptionAdapter, pero aquí es solo el
        # estado INICIAL: get_scene() devolverá algo distinto en cuanto
        # se reporte el primer evento.
        self._scene = initial_scene if initial_scene is not None else Scene.empty()

    def get_scene(self) -> Scene:
        return self._scene

    def report_obstacle(self, obstacle: SphereObstacle) -> None:
        """"Detecta" un obstáculo nuevo: lo añade a la Scene acumulada
        (Scene sigue siendo inmutable -- with_obstacle devuelve una Scene
        nueva, esta clase es la única que muta SU PROPIA referencia a
        "la Scene actual")."""
        self._scene = self._scene.with_obstacle(obstacle)

    def report_object(self, name: str, position: Point) -> None:
        """"Detecta" un objeto/objetivo nuevo (o actualiza uno existente,
        si `name` ya estaba reportado -- Scene.with_object sobrescribe por
        clave)."""
        self._scene = self._scene.with_object(name, position)
