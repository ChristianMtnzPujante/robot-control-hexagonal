"""Primer adaptador de `PlannerSelectionPort` (shared_kernel): devuelve
siempre la misma estrategia, sin mirar la `Scene` recibida -- no hay
todavía ninguna lógica de selección real que implementar (esa es HyperPlan,
ROADMAP.md, Bloque 5). Cierra el puerto en pie: `ControllerNode` puede
llamar a `select(scene)` y usar el resultado como `strategy` de
`_build_adapter` ya desde hoy, aunque la decisión sea trivial.
"""

from __future__ import annotations

from shared_kernel import Scene


class FixedPlannerSelectionAdapter:
    def __init__(self, strategy: str) -> None:
        self._strategy = strategy

    def select(self, scene: Scene) -> str:
        return self._strategy
