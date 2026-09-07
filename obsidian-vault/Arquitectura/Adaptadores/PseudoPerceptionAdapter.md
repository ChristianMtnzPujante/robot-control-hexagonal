---
tags: [arquitectura, adaptador]
---

# PseudoPerceptionAdapter

Implementa [[PerceptionPort]] permitiendo "inyectar" eventos con el
tiempo — un nuevo obstáculo "detectado", un nuevo objetivo — sin cámara
ni fichero, puramente programático. Código:
`src/perception_node/perception_node/adapters/pseudo_perception_adapter.py`.
Primera fase del Bloque 3 ("Pseudo-perceptor"), pensada para poder probar
replanificación real sin depender de que exista percepción de verdad.

## API

`report_obstacle(name, obstacle)` / `report_object(name, position)` —
cada llamada actualiza la `Scene` acumulada internamente (`Scene` sigue
siendo inmutable, `with_obstacle`/`with_object` devuelven una `Scene`
nueva; esta clase es la única que muta SU PROPIA referencia a "la Scene
actual"). Reportar el mismo nombre dos veces actualiza, no duplica.

## Ciclo de vida: independiente de cualquier `ControlSession`

Spike ya resuelto (Vikunja #89, ver [[Estado del Roadmap]]): quien lo usa
(hoy, un demo; en el futuro, un nodo de percepción propio) lo construye y
lo alimenta; `Commander` (o quien construya la sesión) se limita a
ESCUCHAR su `get_scene()`, nunca lo crea ni lo posee de la forma en que
posee una `ControlSession`.

## Auto-descripción

Igual que [[FilePerceptionAdapter]]: `description` declara nombre,
descripción y forma del dato que reporta, al estilo del schema de una
tool de MCP — preparado para el Bloque 6 (LLM vía tools), aunque nadie lo
consuma todavía.

## Ver también

- [[PerceptionPort]]
- [[FilePerceptionAdapter]]
- [[Scene y Percepción]]
