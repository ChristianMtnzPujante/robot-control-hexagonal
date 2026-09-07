---
tags: [arquitectura, puerto]
---

# PerceptionPort

> `get_scene() -> Scene`

`shared_kernel/ports.py` — lo que el sistema sabe de la escena en este
instante: "detecta plano X", "lista obstáculos actuales". Desacoplado de
la implementación de visión — quien consume este puerto
([[PlanningPort]], [[PlannerSelectionPort]]) nunca sabe qué hay detrás.
Devuelve una `Scene` COMPLETA de una vez, no streaming/eventos — ver
[[Scene y Percepción]] para el agregado en sí y cómo `Commander` combina
varios perceptores con `Scene.merge`.

## Adaptadores reales

- **`StaticPerceptionAdapter`**
  (`perception_node/adapters/static_perception_adapter.py`) — una
  `Scene` fija pasada en el constructor. No detecta nada, solo cablea el
  puerto de punta a punta. No depende de `rclpy` (como el resto de
  adaptadores de este puerto) — el nodo ROS2 real que publica en
  `/perception/scene` vive aparte, en `perception_node/node.py`.
- [[FilePerceptionAdapter]] — relee un fichero de texto entero en cada
  `get_scene()`.
- [[PseudoPerceptionAdapter]] — permite "inyectar" eventos con el tiempo,
  sin cámara ni fichero.

## Pendiente (Bloque 3)

- Adaptador de percepción CoppeliaSim (ground truth simulado vía API
  ZMQ) — sin resolver del todo: no hay precedente en el repo de leer el
  RADIO de una esfera vía esa API.
- Cámara real, bloqueado detrás de grounding.

## Ver también

- [[Puertos y Adaptadores]]
- [[Scene y Percepción]]
- [[Estado del Roadmap]]
