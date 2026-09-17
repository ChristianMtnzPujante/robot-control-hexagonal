---
tags: [arquitectura]
---

# Infraestructura ROS2 (ros2_kit)

Referencia función por función de `ros2_kit` — infraestructura compartida
por `robot_node`/`controller_node`/`perception_node`/`commander`, sin que
`shared_kernel` llegue nunca a saber que ROS2 existe (ver
[[Arquitectura Hexagonal]]). El flujo completo YAML → nodo → callback ya
está narrado con ejemplos en [[Anatomía de un Nodo]] — esta nota es el
complemento de referencia, no lo repite.

## `messages.py` — traducción dominio ↔ mensaje ROS2

Único sitio donde se toca `sensor_msgs`/`geometry_msgs`/`std_msgs`
directamente para estas conversiones (antes estaba duplicado en cada
`node.py`).

- `to_joint_configuration(msg) -> JointConfiguration` / `to_joint_state_msg(configuration) -> JointState` —
  vector de nombre+ángulo en ambas direcciones. La construcción usa
  `JointConfiguration.create(...)` y relanza como `ValueError` si el
  `Either` viene `left` (nombres duplicados/vacíos) — un mensaje ROS2
  corrupto no debe colar un value object inválido en el dominio.
- `to_pose(msg) -> Pose` / `to_pose_msg(pose) -> PoseMsg` — ejemplo
  completo del patrón en [[Anatomía de un Nodo]].
- `to_scene_msg(scene) -> String` / `from_scene_msg(msg) -> Scene` —
  serializan `Scene` entera como JSON dentro de `std_msgs/String`, sin
  paquete de interfaces `.msg` propio (mismo patrón que `<ns>/feedback`).
  Usan dos helpers privados de ida y vuelta, `_point_to_list`/
  `_point_from_list`, para no repetir `[x, y, z]` en las tres colecciones
  de `Scene` (`planes`, `obstacles`, `objects`).
- **(08/09)** `to_obstacle_report_msg(name, obstacle) -> String` /
  `from_obstacle_report_msg(msg) -> (name, SphereObstacle)` y
  `to_object_report_msg(name, point) -> String` /
  `from_object_report_msg(msg) -> (name, Point)` — mismo patrón JSON que
  `to_scene_msg`, pero para UN SOLO evento con nombre en vez de la `Scene`
  completa. Reutilizan `_point_to_list`/`_point_from_list`. Es el mensaje
  de los dos topics nuevos de [[PerceptionNode]]
  (`/perception/report_obstacle`/`report_object`) que alimentan
  [[PseudoPerceptionAdapter]] desde fuera de su propio proceso.

## `node_config.py` — config declarativa YAML → rclpy

Dos funciones con responsabilidades que el propio docstring del módulo
marca como deliberadamente separadas:

- `load_node_config(path) -> NodeConfig` — solo lee el YAML y resuelve
  nombres a objetos reales (`"pkg/msg/Tipo"` → la clase de mensaje vía
  `rosidl_runtime_py.get_message`, nombre de QoS → `QoSProfile`). No toca
  `rclpy` — puede llamarse antes de que exista el `Node`, porque hace
  falta leer el propio `node_name` del YAML para poder construirlo.
- `apply_node_config(node, config) -> Dict[str, Publisher]` — al revés,
  solo llama a `rclpy` de verdad sobre un `node` ya construido, en un
  orden concreto y con motivo: (1) parámetros primero, porque el período
  de un timer o el target de un adaptador puede depender de su valor ya
  declarado; (2) publishers, devueltos en un dict indexado por topic para
  que el nodo los guarde como quiera; (3) subscriptions, resolviendo el
  callback por nombre con `getattr(node, ...)` — si no hay `callback`
  declarado, lanza `ValueError` explícito en vez de suscribir sin
  handler; (4) timers, leyendo el período del **valor ya declarado** del
  parámetro (no un literal aparte), para que siga siendo el mismo número
  que ve `ros2 param get`.
- `_build_descriptor(spec) -> ParameterDescriptor` — traduce el `range:`
  del YAML a un `ParameterDescriptor` real; sólo admite `range` en
  parámetros `double`/`int` (`ValueError` en cualquier otro tipo). El
  rechazo de un valor fuera de rango lo hace `rclpy` en runtime, no una
  validación propia.
- `_resolve_topic(spec) -> TopicSpec` — resuelve `message_type` con la
  misma función que usa `ros2 topic pub`/`ros2 topic echo` por dentro
  (`rosidl`), no una tabla propia.
- `_resolve_qos(value)` — un YAML `qos: 10` (entero) se trata como
  profundidad de cola simple; un string se busca en `_QOS_BY_NAME`
  (`GOAL_QOS`/`STRATEGY_QOS`/`SCENE_QOS`) y lanza `ValueError` con la
  lista de perfiles válidos si no existe. El YAML referencia un perfil
  *por nombre* — nunca redefine reliability/durability desde datos, eso
  sigue siendo una decisión de código (ver `qos.py` abajo).
- `package_config_path(package_name, filename) -> str` — ruta a un YAML
  instalado en `share/<paquete>/config/`, mismo mecanismo que cada
  paquete ya usa para `resource/` en su `setup.py`.

## `qos.py` — perfiles compartidos, cada uno con un motivo real

Los tres perfiles nacen de condiciones de carrera concretas encontradas
al cablear el sistema — no son un catálogo genérico:

- **`GOAL_QOS`** (`RELIABLE` + `TRANSIENT_LOCAL`) — `Commander` publica
  el goal sin esperar a que `controller_node` ya esté suscrito (arrancan
  como procesos aparte, en paralelo). Con la QoS por defecto (volátil),
  publicar antes de que el suscriptor se haya descubierto pierde el
  mensaje para siempre, sin error ni reintento. `TRANSIENT_LOCAL` retiene
  el último mensaje y lo entrega también a quien se suscriba después, así
  que el orden de arranque deja de importar.
- **`STRATEGY_QOS`** (`BEST_EFFORT` + `VOLATILE`) — la contraria a
  propósito: perder un cambio de estrategia no es crítico (la sesión
  sigue con la estrategia anterior, que sigue siendo válida), así que no
  se paga el coste de acks/reintentos de `RELIABLE` para un canal de
  control best-effort.
- **`SCENE_QOS`** (`RELIABLE` + `TRANSIENT_LOCAL`) — mismo razonamiento
  que `GOAL_QOS`, mismo problema de arranque: `perception_node` y quien
  lo escuche (`Commander`) son procesos separados sin coordinación de
  arranque. Sin retención, `Commander` se queda sin `Scene` hasta el
  siguiente ciclo del timer del perceptor — o para siempre, si el
  perceptor solo publica una vez.

## `runner.py` — ciclo de vida rclpy, una sola vez

- `run_node(node_factory, args=None)` — caso simple: `rclpy.init` →
  construir el nodo → `rclpy.spin` → `shutdown_node` en un `finally`
  (para que el `spin` interrumpido por Ctrl+C no deje el nodo sin cerrar).
- `shutdown_node(node)` — solo el apagado (`destroy_node` +
  `rclpy.shutdown`), separado de `run_node` para casos como `Commander`,
  que necesita hacer algo (crear `ControlSession`s, mandar un goal) entre
  construir el nodo y llamar a `spin`, y por tanto no puede usar
  `run_node` tal cual.

## `ros1_kit/bridge.py` — boceto descartado, sin conectar

`Ros1Bridge`/`Ros1BridgeConfig` encapsularían el ciclo de vida de un
proceso `ros1_bridge` (mismo papel que `ControlSession` cumple para
`robot_node`/`controller_node`, pero para el puente en sí) — pensado para
cuando conectar el CR5 real fuera vía su driver oficial `dobot_bringup`
(ROS1/catkin). `start()` lanza `NotImplementedError` a propósito: requiere
ROS1 Noetic conviviendo con ROS2 Humble en la misma máquina, no disponible
en este entorno. Descartado el 04/09 a favor de reimplementar el
protocolo TCP/IP directo — ver [[Decisiones de Diseño Clave]] y
[[Cr5RealRobotAdapter]] (ya verificado contra el robot físico sin
necesidad de este puente).

## Ver también

- [[Anatomía de un Nodo]]
- [[Arquitectura Hexagonal]]
- [[Commander y ControlSession]]
- [[Decisiones de Diseño Clave]]
