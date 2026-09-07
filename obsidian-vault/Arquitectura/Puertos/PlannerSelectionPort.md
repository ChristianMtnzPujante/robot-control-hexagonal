---
tags: [arquitectura, puerto]
---

# PlannerSelectionPort

> `select(scene: Scene) -> str`

`shared_kernel/ports.py` — elige qué estrategia de planificación usar
según el estado de la escena (features derivadas de `Scene`, al estilo
HyperPlan — Bloque 5, sin implementar todavía). Devuelve el identificador
de estrategia que ya consume el registro `str -> factoría` de
`controller_node._build_adapter` (p. ej. `"chomp"`, `"rrt"`) — mismo
mecanismo que ya usan [[KinematicsPort]]/[[PlanningPort]].

## Adaptador

- **`FixedPlannerSelectionAdapter`**
  (`controller_node/adapters/fixed_planner_selection_adapter.py`) — el
  único de hoy. Devuelve siempre la MISMA estrategia (pasada en el
  constructor), sin mirar la `Scene` recibida. No hay ninguna lógica de
  selección real que implementar todavía (eso es HyperPlan) — este
  adaptador cierra el puerto en pie: `ControllerNode` ya puede llamar a
  `select(scene)` y usar el resultado como `strategy`, aunque la decisión
  sea trivial.

## Ver también

- [[Puertos y Adaptadores]]
- [[PlanningPort]]
- [[Estado del Roadmap]]
