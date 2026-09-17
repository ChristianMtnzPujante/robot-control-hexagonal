---
tags: [arquitectura, nodo]
---

# PerceptionNode

Referencia función por función de
`src/perception_node/perception_node/node.py`. Para el patrón genérico
YAML→callback, ver [[Anatomía de un Nodo]].

Envuelve un [[PerceptionPort]] — mismo papel que `robot_node`/
`controller_node` para sus puertos: no decide nada, solo traduce
(`to_scene_msg`) y publica periódicamente lo que el adaptador reporte.

## Ciclo de vida: sin namespace de sesión

A diferencia de `robot_node`/`controller_node`, este nodo NO vive dentro
del namespace de ninguna `ControlSession` — tiene vida propia y publica en
un topic GLOBAL, `_SCENE_TOPIC = "/perception/scene"`, del que `Commander`
escucha desde fuera de cualquier sesión concreta (ver
[[Scene y Percepción]] y `docs/nodos_ros2.md` §4, Bloque 3).

## Registro de adaptadores — SIN el patrón `_TARGETS`/factoría

A diferencia de `robot_node`/`controller_node`, `_build_adapter` aquí es
un `if/elif` directo sobre `perception_target`, no un dict de factorías:

| `perception_target` | Adaptador |
|---|---|
| `"fichero"` | [[FilePerceptionAdapter]] — exige `file_path`, si no `ValueError` |
| `"estatico"` | `StaticPerceptionAdapter(Scene.empty())` — siempre una escena vacía, no configurable por parámetro |
| `"pseudo"` | [[PseudoPerceptionAdapter]] — **(08/09)** ahora sí está en el registro, ver más abajo |

## Inyección de eventos por ROS2 — `_on_report_obstacle`/`_on_report_object` (08/09)

Dos subscriptions nuevas, declaradas incondicionalmente en el YAML (igual
que el resto — `apply_node_config` no sabe todavía qué `perception_target`
se eligió cuando las crea):

- `/perception/report_obstacle` → `_on_report_obstacle`
- `/perception/report_object` → `_on_report_object`

Cada callback deserializa el mensaje (`from_obstacle_report_msg`/
`from_object_report_msg`, JSON en `std_msgs/String`, mismo patrón que
`to_scene_msg`) y llama, por duck typing (`getattr(self._perception,
"report_obstacle", None)`, no `isinstance`), al método correspondiente del
adaptador activo. Con `perception_target` distinto de `"pseudo"` el
adaptador no tiene ese método — el callback registra un `warning` y no
hace nada, en vez de lanzar una excepción: las subscriptions existen
siempre, aunque solo un `perception_target` sepa aprovecharlas.

QoS `GOAL_QOS` (no `SCENE_QOS`): un reporte es una orden puntual como
`<ns>/goal`, no el estado retenido de la escena completa — mismo problema
de arranque (publicador y suscriptor son procesos separados que no se
coordinan para arrancar en orden).

Antes del 08/09, [[PseudoPerceptionAdapter]] solo podía alimentarse
llamando a sus métodos directamente desde el mismo proceso Python (un
demo) — ahora un proceso externo (otro nodo, `ros2 topic pub`, en el
futuro un detector real) puede inyectar eventos sin compartir proceso con
este nodo. Verificado en vivo con `rclpy` real, no solo con tests.

## `_publish_scene()`

Llama a `get_scene()` del adaptador y publica el resultado en
`_SCENE_TOPIC`, sin transformar nada — toda la lógica de "qué hay en la
escena" vive en el adaptador, nunca aquí.

## Ver también

- [[PerceptionPort]]
- [[FilePerceptionAdapter]]
- [[PseudoPerceptionAdapter]]
- [[Scene y Percepción]]
- [[Anatomía de un Nodo]]
- [[Infraestructura ROS2 (ros2_kit)]] — `to_obstacle_report_msg`/`from_obstacle_report_msg` y sus equivalentes de objeto
