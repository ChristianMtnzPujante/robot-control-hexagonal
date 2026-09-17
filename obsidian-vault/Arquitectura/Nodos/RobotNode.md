---
tags: [arquitectura, nodo]
---

# RobotNode

Referencia función por función de `src/robot_node/robot_node/node.py`. Para
el patrón genérico YAML→callback que sigue (y de dónde sale cada pieza),
ver [[Anatomía de un Nodo]] — esta nota es solo lo específico de ESTE
nodo.

Envuelve un [[RobotConnectorPort]]: no decide nada, solo traduce mensajes
ROS2 ↔ dominio y delega en el adaptador concreto elegido por parámetro —
la misma imagen ejecutable sirve para simulado o real sin recompilar.

## Registro `_TARGETS` — robot_target → adaptador

| `robot_target` | Adaptador | Parámetros que de verdad usa |
|---|---|---|
| `"simulado"` | [[CoppeliaSimRobotAdapter]] | `joint_names`, `tip_name`, `scene_path`, `zmq_port` |
| `"real"` | [[Cr5RealRobotAdapter]] | `joint_names`, `cr5_host`, `cr5_movj_cp`, `cr5_joint_limits_degrees` |

Todas las factorías reciben las MISMAS siete cosas (las que no necesita,
las ignora) — el propio comentario del código señala el límite de este
patrón: quita el `if/elif`, pero no resuelve que un robot real futuro
pueda necesitar un dato de conexión/ajuste que estas siete no cubran.
Añadir un robot nuevo es añadir una entrada aquí, no una rama nueva (ver
[[Estado del Roadmap]], Bloque 9, y [[Conectar un Robot Nuevo]]).

`joint_names` llega ya resuelto (literal o derivado de un URDF) por quien
lanzó el proceso — `robot_node` no sabe ni necesita saber de dónde salió,
ver `ControlSession._resolve_joint_names` en [[Commander y ControlSession]].

## `__init__` / `_build_adapter`

Orden fijo: leer el YAML → `super().__init__(config.node_name)` → declarar
parámetros/publishers/subscripciones (`apply_node_config`) → leer los
parámetros del robot → construir el adaptador. `_build_adapter` solo hace
el lookup en `_TARGETS` y lanza `ValueError` si `robot_target` no existe
en el registro.

## `_on_joint_command(msg)`

Traduce el `JointState` a `JointConfiguration` y llama a
`set_joints`. Captura `RobotConnectorError`: **corregido 07/09** — antes
un solo fallo de protocolo (visto en vivo: reset de conexión a mitad de
una trayectoria real) tumbaba el nodo entero. `Cr5CommandSocket` ya
reconecta y reintenta sola una vez antes de llegar aquí (ver
[[Cr5RealRobotAdapter]]), así que si este error se ve es que ya se agotó
ese margen — se loggea y el nodo sigue vivo, pero ese waypoint concreto
se pierde sin reintentarse desde aquí (política de reintento/degradación
todavía sin decidir del todo).

## `_on_goal(msg)`

Solo tiene efecto si el adaptador ofrece `mark_goal` (duck typing,
`getattr(..., None)`) — decoración puramente cosmética de
[[CoppeliaSimRobotAdapter]], no parte formal de [[RobotConnectorPort]].
Contra el CR5 real, este callback no hace nada.

## `_publish_state()`

Corre en un timer rápido (`state_publish_period_seconds`, default 50ms)
contra el mismo canal de red que puede fallar. Mismo criterio que
`_on_joint_command`: captura `RobotConnectorError`, se salta esa
publicación y reintenta en el siguiente tick — sin este try/except, un
hipo leyendo estado tumbaría el nodo tan fácil como uno enviando un
comando.

## `destroy_node()`

**Corregido 07/09**: antes `robot_node` no cerraba/des-energizaba el
adaptador en ningún camino de cierre (ni normal — `ControlSession.stop()`
manda SIGTERM y `rclpy.spin()` retorna vía el `try/finally` de
`ros2_kit.run_node` — ni por crash). Ahora llama a `close()` del
adaptador; un fallo cerrando se loggea pero no impide el resto del
cierre. Ver [[Commander y ControlSession]] para el hallazgo de proceso
zombie relacionado del mismo día.

## Ver también

- [[RobotConnectorPort]]
- [[Cr5RealRobotAdapter]]
- [[CoppeliaSimRobotAdapter]]
- [[Anatomía de un Nodo]]
- [[Commander y ControlSession]]
