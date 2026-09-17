---
tags: [arquitectura, nodo]
---

# Commander (referencia de código)

Referencia función por función de `commander_node.py` (clase `Commander`) y
`control_session.py` (clase `ControlSession`). El rol de cada clase, el
diagrama de topics y el hallazgo del proceso zombie (07/09) ya están
narrados en [[Commander y ControlSession]] — esta nota es el complemento:
qué hace cada método concreto.

## `Commander` (`commander/commander_node.py`)

- **`__init__`** — guarda `_sessions` (nombre → `ControlSession`),
  `_goal_publishers` (nombre → publisher de `<ns>/goal`) y
  `_last_perceived_goal` (nombre → último `Point` reenviado desde
  percepción, para no republicar el mismo objetivo en cada ciclo del timer
  de `perception_node`, que republica su última `Scene` conocida aunque
  nada haya cambiado).
- **`create_session(...)`** — construye y arranca una `ControlSession` con
  el namespace `/session_<name>`, la guarda en `_sessions`, y crea el
  publisher de `<ns>/goal` y la suscripción a `<ns>/feedback` para esa
  sesión. Recibe literalmente todos los parámetros de conexión posibles
  (CR5, URDF, naive-test, CoppeliaSim...) y se los pasa tal cual a
  `ControlSession` — no decide nada, solo los reenvía.
- **`_on_feedback(session_name, msg)`** — deserializa el JSON de
  `<ns>/feedback` y lo loguea con el prefijo `[session_name]`. No hace
  nada más con el contenido — es solo el sumidero de logging del feedback
  de esa sesión.
- **`send_goal(session_name, goal)`** — publica un `Pose` de dominio (ya
  convertido a `PoseMsg`) en el publisher de esa sesión. Punto de entrada
  único para mandar un objetivo cartesiano, tanto a mano como desde
  `_on_perceived_scene`.
- **`follow_perception(session_name, topic=...)`** — suscribe la sesión al
  topic global `/perception/scene` (fuera de cualquier namespace de
  sesión). Cablea la suscripción; la decisión de qué hacer con cada
  `Scene` recibida vive en `_on_perceived_scene`.
- **`_on_perceived_scene(session_name, msg)`** — deserializa la `Scene`
  (`from_scene_msg`), busca `Scene.objects["objetivo"]`; si no hay
  objetivo, o es el mismo `Point` que la última vez reenviado a esa
  sesión, no hace nada. Si es nuevo, construye un `Pose` con orientación
  identidad (`Scene.objects` solo guarda posición) y lo manda con
  `send_goal`. Es la única lógica de decisión de todo `Commander` — y es
  fusión de datos, no planificación: `Commander` sigue sin saber qué
  estrategia usa `controller_node`.
- **`close_session(session_name)`** — para la sesión (`ControlSession.stop`)
  y limpia sus tres entradas de estado (`_sessions`, `_goal_publishers`,
  `_last_perceived_goal`).
- **`main()`** — demo mínima hardcodeada: una sesión `coppeliasim_ik` +
  simulado, un único `send_goal` cercano a la posición actual (simIK
  resuelve por iteración local — un salto grande puede no converger, ver
  [[CoppeliaSimIkKinematicsAdapter]]), y `rclpy.spin` hasta cerrar. No es
  una limitación de la API — `joint_names`/`tip_name` ya son parámetros de
  `create_session` — es solo el único demo que vive en este archivo.

## `ControlSession` (`commander/control_session.py`)

- **`__init__`** — guarda todos los parámetros de conexión recibidos de
  `Commander` como atributos privados; resuelve `_joint_names` de
  inmediato vía `_resolve_joint_names` (antes de lanzar ningún proceso).
  Todo parámetro opcional vacío/`None` significa "usa el default del YAML
  del nodo correspondiente" — no se manda por `-p` si no se pide
  explícitamente (`-p x:=""` rompe el parseo de `ros2 run`, ver `start`).
- **`_resolve_joint_names`** — ya narrado en detalle en
  [[Commander y ControlSession]] (deriva `joint_names` de un URDF antes de
  lanzar ningún proceso ROS2, en vez de declarar-y-corregir dentro del
  nodo). Lanza `ValueError` si se da `urdf_path` sin `base_link`/`tip_link`.
- **`start()`** — construye los `argv` de `ros2 run robot_node` y
  `ros2 run controller_node` (namespace, `joint_names`, y cada parámetro
  opcional solo si tiene valor) y lanza ambos como procesos reales
  (`subprocess.Popen(..., start_new_session=True)`) — líderes de su propio
  grupo de procesos, condición necesaria para que `stop()` pueda señalar
  al grupo entero y no solo al lanzador de `ros2 run`.
- **`stop()`** — llama a `_terminate_process_group` sobre `robot_process` y
  `controller_process` (si existen).
- **`_terminate_process_group`** — el fix del 07/09 (`e009321`), narrado en
  detalle en [[Commander y ControlSession]]: `ros2 run` no hace `exec()`,
  así que el PID de `Popen` es el del lanzador, no el del nodo `rclpy`
  real. Señala `SIGTERM` al grupo de procesos entero (`os.killpg`, PID
  negativo — la pertenencia a un grupo no cambia al reparentar a `init`),
  espera 5s, y escala a `SIGKILL` si no ha terminado.
- **`__enter__`/`__exit__`** — azúcar de context manager sobre
  `start()`/`stop()`; permite `with ControlSession(...) as session:`.

## Ver también

- [[Commander y ControlSession]]
- [[Puertos y Adaptadores]]
- [[Herramientas de CoppeliaSim]]
- [[Estado del Roadmap]]
