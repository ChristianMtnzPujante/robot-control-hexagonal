---
tags: [arquitectura, nodo]
---

# ControllerNode

Referencia función por función de
`src/controller_node/controller_node/node.py`. Para el patrón genérico
YAML→callback, ver [[Anatomía de un Nodo]] — esta nota es lo específico de
ESTE nodo.

Envuelve un [[KinematicsPort]] (y, indirectamente, decide cuál usar — hoy
no hay [[PlanningPort]] ni [[PlannerSelectionPort]] cableados en este
nodo, ver [[Estado del Roadmap]]): recibe un objetivo cartesiano, calcula
la trayectoria con la estrategia elegida y va publicando los waypoints a
`robot_node`, reportando progreso al Commander por `feedback`.

## Registro `_STRATEGIES` — strategy → adaptador

| `strategy` | Adaptador | Usa `RobotDescription` |
|---|---|---|
| `"poe"` | [[PoeKinematicsAdapter]] | Sí, si hay `urdf_path` — si no, default hardcodeado del CR5 |
| `"ga"` | `GaKinematicsAdapter` (stub) | Sí |
| `"dh"` | `DhKinematicsAdapter` (stub) | No |
| `"naive_test"` | `NaiveTestKinematicsAdapter` | No — usa `naive_test_amplitude_radians`/`naive_test_steps` |
| `"straight_line"` | `StraightLineKinematicsAdapter` | No |
| `"coppeliasim_ik"` | [[CoppeliaSimIkKinematicsAdapter]] | No — usa `zmq_port` |

Detalle de los stubs/dobles en [[KinematicsPort]]. Cada factoría recibe el
propio nodo (para leer lo que necesite) y el `RobotDescription` ya
resuelto o `None` — la que no lo usa lo ignora. Añadir una estrategia
nueva es una entrada más aquí, no tocar `_build_adapter` (Bloque 9).

## Cambio de estrategia EN CALIENTE — lo que distingue a este nodo

A diferencia de `robot_node`/`perception_node`, la estrategia no queda
fija al arrancar: el topic `set_strategy` (`_on_set_strategy`) permite
cambiarla desde cualquier otro nodo, local o remoto (ROS2/DDS no
distingue), sin relanzar la sesión (Bloque 7). Solo afecta al PRÓXIMO
objetivo — una trayectoria ya en curso (`_pending_waypoints`) sigue
drenándose con la estrategia con la que se calculó; cancelarla a medio
camino sería "replanificación reactiva" de verdad (Bloque 4), que este
mecanismo no resuelve.

## `_build_adapter(strategy)` / `_load_robot_description()`

`_build_adapter` relee `zmq_port`/`urdf_path`/`base_link`/`tip_link` de
`self` (no cambian entre llamadas) para poder reconstruirse desde
`_on_set_strategy` sin pedir de nuevo esos datos por mensaje.
`_load_robot_description` devuelve `None` si no hay `urdf_path` (cada
adaptador usa entonces su propio default) y lanza `ValueError` si hay
`urdf_path` pero falta `base_link` o `tip_link` — no aplica a
`"dh"`/`"naive_test"`/`"straight_line"`/`"coppeliasim_ik"`, que no leen
`RobotDescription` en absoluto.

## `_on_joint_states(msg)`

Guarda la configuración actual y, si había un `_pending_goal` esperando,
lo procesa ahora (`_start_trajectory`) y lo limpia.

## `_on_goal(msg)`

Si todavía no llegó ningún `joint_states`, guarda el objetivo en
`_pending_goal` en vez de descartarlo (se resuelve en cuanto llegue el
primero, ver `_on_joint_states`) y publica feedback
`"esperando_estado_robot"`. Junto con `GOAL_QOS` (evita perder el mensaje
si `Commander` publica antes de que este nodo se haya suscrito), esto
elimina del todo la condición de carrera de arranque entre
commander/controller_node/robot_node, sin importar en qué orden arranquen.

## `_on_set_strategy(msg)`

Intenta reconstruir el adaptador con la estrategia nueva. Si falla
(estrategia desconocida, `urdf_path` que ya no existe...) **no propaga el
error** — la sesión sigue con la estrategia anterior, que sigue siendo
válida (best-effort, ver `STRATEGY_QOS`); publica feedback
`"estrategia_invalida"` con el motivo. Límite deliberado: un mensaje
externo puede pedir cualquier cosa, y el nodo no debe morir por ello.

## `_start_trajectory(goal)` / `_advance_trajectory()`

`_start_trajectory` publica `"calculando"`, llama a
`compute_trajectory` y guarda los waypoints resultantes en
`_pending_waypoints`, publicando `"trayectoria_calculada"` con el conteo.
`_advance_trajectory` (llamado desde un timer, ver `node_config`) hace
`pop(0)` de la cola, publica ese waypoint a `robot_node` y reporta
`"waypoint_enviado"` con los restantes; al vaciarse publica
`"completado"`.

## Ver también

- [[KinematicsPort]]
- [[PoeKinematicsAdapter]]
- [[CoppeliaSimIkKinematicsAdapter]]
- [[Anatomía de un Nodo]]
- [[Estado del Roadmap]]
