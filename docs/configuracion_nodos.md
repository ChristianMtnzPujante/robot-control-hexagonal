# Configuración práctica de los nodos ROS2

Referencia de uso: qué parámetros acepta cada nodo, con qué valor por
defecto, y cómo se sobreescriben en la práctica. Es el complemento
práctico de `docs/nodos_ros2.md` (que define el CONTRATO entre nodos,
responsabilidades y topics) — aquí solo hay tablas para consultar antes de
lanzar algo.

Cada paquete de nodo tiene además su propio `README.md`
(`src/robot_node/README.md`, `src/controller_node/README.md`,
`src/perception_node/README.md`) con ideas de qué MÁS suele hacer falta
configurar en un nodo de ese tipo, aunque no esté implementado hoy — por
ejemplo, límites por articulación en `robot_node`.

## El mecanismo: un YAML por nodo, no parámetros sueltos en código

Desde `ros2_kit/node_config.py`, cada nodo (`robot_node`, `controller_node`,
`perception_node`) declara sus parámetros/topics/timers en un YAML propio,
instalado en `share/<paquete>/config/<paquete>.yaml`:

```
src/robot_node/config/robot_node.yaml
src/controller_node/config/controller_node.yaml
src/perception_node/config/perception_node.yaml
```

El `__init__` de cada nodo solo hace `load_node_config(...)` +
`apply_node_config(self, config)` — declarar un parámetro nuevo, o
cambiar su valor por defecto, es editar el YAML, no tocar `node.py`. Lo
que el YAML **no** contiene es lógica: los callbacks (`_on_goal`,
`_publish_state`...) siguen siendo métodos normales de la clase del nodo;
el YAML solo los referencia por nombre.

`Commander` es la excepción — no tiene YAML propio porque no declara
ningún parámetro ROS2: su "configuración" es la firma de
`Commander.create_session(...)`, en Python (ver más abajo).

## Cómo sobreescribir un valor

Tres formas, de más puntual a más permanente:

1. **Al lanzar el nodo suelto**, por CLI:
   ```bash
   ros2 run controller_node controller_node --ros-args -p strategy:=poe -p waypoint_period_seconds:=1.0
   ```
2. **En caliente**, sobre un nodo ya corriendo (si el parámetro no es
   `read_only` — ninguno de los definidos hoy lo es):
   ```bash
   ros2 param set /controller_node waypoint_period_seconds 1.0
   ```
   Si el valor viola el `range` declarado (ver tablas abajo), ROS2 lo
   rechaza en el momento — no hace falta validación propia:
   ```
   $ ros2 param set /controller_node waypoint_period_seconds 999.0
   Setting parameter failed: Parameter waypoint_period_seconds out of range Min: 0.05, Max: 5.0, value: 999.0
   ```
3. **Vía `Commander.create_session(...)`** — el camino normal cuando el
   nodo lo lanza una `ControlSession`, no tú a mano (ver última sección).

Para ver qué tiene declarado un nodo ya corriendo, sin mirar el YAML:
```bash
ros2 param list /controller_node
ros2 param get /controller_node strategy
ros2 param describe /controller_node waypoint_period_seconds   # incluye el range, si lo tiene
```

---

## `robot_node`

Adaptador de `RobotControllerPort` — ejecuta `joint_command`, reporta
`joint_states`. YAML: `src/robot_node/config/robot_node.yaml`.

| Parámetro | Tipo | Default | Rango | Significado |
|---|---|---|---|---|
| `robot_target` | string | `simulado` | — | `"simulado"` → `CoppeliaSimRobotAdapter`; `"real"` → `Cr5RealRobotAdapter` (host/puerto TCP hardcodeados en el adaptador, no configurables por parámetro hoy). |
| `joint_names` | string[] | `[joint1..joint6]` | — | Nombres de las articulaciones, en orden. Para `robot_node` es un valor **literal** — de dónde sale (a mano, o derivado de un URDF) lo decide quien lo lanza, no este nodo (ver la sección de `Commander` más abajo). Por CLI: `-p joint_names:="[joint1,joint2,...]"`. |
| `state_publish_period_seconds` | double | `0.05` | `0.01–5.0` | Cada cuánto se publica `joint_states`. |
| `tip_name` | string | `""` (vacío) | — | Nombre del dummy "tip" en la escena CoppeliaSim, solo decorativo (trail de waypoints). Vacío lo desactiva. |
| `scene_path` | string | `""` (vacío) | — | Ruta a un `.ttt` a cargar y poner en play al arrancar. Vacío asume que ya hay una escena abierta. |
| `zmq_port` | int | `23000` | — | Puerto ZMQ de la instancia de CoppeliaSim a la que conectar (varias instancias = varios puertos, ver `commander_demo_two_sessions`). |

**Topics:** se suscribe a `joint_command` (`sensor_msgs/JointState`) y a
`goal` (`geometry_msgs/Pose`, solo para el `mark_goal` decorativo si el
adaptador lo ofrece); publica `joint_states` (`sensor_msgs/JointState`).

---

## `controller_node`

Adaptador de `KinematicsPort`/`PlanningPort` — de un `goal` cartesiano a
una `Trajectory`, enviada como `joint_command`. YAML:
`src/controller_node/config/controller_node.yaml`.

| Parámetro | Tipo | Default | Rango | Significado |
|---|---|---|---|---|
| `strategy` | string | `naive_test` | — | `poe`\|`ga`\|`dh`\|`naive_test`\|`straight_line`\|`coppeliasim_ik`. Cambiable en caliente publicando en el topic `set_strategy`, no solo al arrancar (ver `docs/nodos_ros2.md` §1.3 — el YAML de `ros2 param set` NO cambia la estrategia, solo el topic). |
| `waypoint_period_seconds` | double | `0.5` | `0.05–5.0` | Cada cuánto se envía el siguiente waypoint de la trayectoria calculada. |
| `zmq_port` | int | `23000` | — | Solo lo usa la estrategia `coppeliasim_ik`. |
| `urdf_path` | string | `""` (vacío) | — | Ruta a un `.urdf` real, para construir un `RobotDescription` genérico. Solo lo usan `poe`/`ga`. Vacío conserva el CR5 hardcodeado que cada adaptador trae por defecto. |
| `base_link` | string | `""` (vacío) | — | Obligatorio junto con `urdf_path` (la cadena serie a extraer). |
| `tip_link` | string | `""` (vacío) | — | Igual que `base_link`. |

**Topics:** se suscribe a `goal` (`geometry_msgs/Pose`, `GOAL_QOS`), a
`set_strategy` (`std_msgs/String`, `STRATEGY_QOS` — el mecanismo real de
cambio de estrategia en caliente) y a `joint_states`
(`sensor_msgs/JointState`); publica `joint_command`
(`sensor_msgs/JointState`) y `feedback` (`std_msgs/String`, JSON).

---

## `perception_node`

Adaptador de `PerceptionPort` — publica periódicamente la `Scene` que
reporte el adaptador elegido. A diferencia de los dos anteriores, **no
vive dentro del namespace de ninguna `ControlSession`** — es un proceso
con vida propia, publicando en un topic global (ver `docs/nodos_ros2.md`
§4). YAML: `src/perception_node/config/perception_node.yaml`.

| Parámetro | Tipo | Default | Rango | Significado |
|---|---|---|---|---|
| `perception_target` | string | `fichero` | — | `"fichero"` → `FilePerceptionAdapter` (lee `file_path`); `"estatico"` → `StaticPerceptionAdapter` (una `Scene` vacía). |
| `file_path` | string | `""` (vacío) | — | Obligatorio si `perception_target=fichero` — el nodo lanza `ValueError` al arrancar si falta. |
| `scene_publish_period_seconds` | double | `0.5` | `0.05–10.0` | Cada cuánto se republica la última `Scene` conocida. |

**Topics:** publica `/perception/scene` (`std_msgs/String`, JSON vía
`to_scene_msg`, `SCENE_QOS`) — ruta absoluta a propósito, no relativa a
ningún namespace de sesión.

---

## `Commander` — configuración vía código, no YAML

`Commander` no declara ningún parámetro ROS2 propio: cada `ControlSession`
que crea lanza `robot_node`/`controller_node` como procesos nuevos
(`ros2 run ... --ros-args -p ...`), traduciendo los argumentos de
`create_session(...)` directamente a los parámetros de la tabla de
arriba (ver `commander/control_session.py`, método `start()`):

```python
commander.create_session(
    name="demo",
    robot_target="simulado",          # -> robot_node: robot_target
    controller_strategy="poe",        # -> controller_node: strategy
    joint_names=[...],                # -> ambos: joint_names (ver derivación abajo)
    waypoint_period_seconds=1.0,      # -> controller_node: waypoint_period_seconds
    zmq_port=23000,                   # -> ambos: zmq_port
    tip_name="Link6_visual",          # -> robot_node: tip_name
    scene_path="cr5_base.ttt",        # -> robot_node: scene_path
    urdf_path="...", base_link="...", tip_link="...",  # -> controller_node + derivar joint_names
)
```

**`joint_names` puede venir derivada de un URDF, resuelto en `ControlSession`, no en los nodos.**
Si `urdf_path` está presente, `ControlSession._resolve_joint_names` (en
`control_session.py`, llamado desde `__init__`, antes de lanzar ningún
proceso) sustituye el `joint_names` que le hayas pasado por
`[j.name for j in RobotDescription.joints]` — en Python normal, sin tocar
`rclpy` para nada. Por eso el `joint_names` que reciben `robot_node` y
`controller_node` por `-p` ya llega resuelto: ninguno de los dos nodos
sabe ni necesita saber que salió de un URDF. Esta es la razón concreta de
que la derivación NO viva dentro de `robot_node`: `urdf_path` sería, si
fuera parámetro suyo, overrideable por `-p` al lanzarlo — y ROS2 solo
resuelve ese override una vez el propio `Node` ya existe, así que no se
podría derivar nada antes de declarar sus parámetros. `ControlSession`, al
ser código Python que construye la línea de arranque, no tiene esa
restricción.

Nota: `control_session.py` también reenvía `tip_name`/`scene_path` a
`controller_node`, pero el YAML de `controller_node` no declara esos dos
parámetros (no los necesita) — ROS2 los ignora en silencio si llegan sin
`declare_parameter` correspondiente. No es un fallo, solo un argumento de
más que hoy no hace nada en ese proceso.

`robot_target`, `controller_strategy` y `joint_names` son obligatorios en
`create_session` (sin default) — el resto de argumentos sí tienen default,
y coinciden con los del YAML del nodo correspondiente: si no los pasas,
cae al mismo valor que tendría el nodo lanzado suelto.

## Tareas del programador al configurar/extender un nodo

El grueso del trabajo se reparte en dos sitios, y son de naturaleza
distinta — uno es dato declarativo, el otro es código de dominio:

1. **Escribir/editar el YAML** (`parameters:`, `publishers:`,
   `subscriptions:` o `timers:` — ver `ros2_kit/node_config.py` para el
   esquema completo, incluye `range` para parámetros numéricos). Aquí solo
   se declara QUÉ existe (nombre, tipo, QoS, valor por defecto), nunca CÓMO
   se comporta.
2. **Programar el/los callback(s)** — el método real de la clase del nodo
   que el YAML referencia por nombre (`callback: _on_goal`). Aquí vive toda
   la lógica de dominio: qué hacer con el mensaje que llega, qué publicar
   en respuesta, qué estado actualizar.

Pero no son las únicas dos tareas — estas otras son fáciles de olvidar
porque no tienen su propio apartado en `node_config.py`, y alguna ya nos
hizo perder tiempo al construir esto:

3. **Que el tipo de mensaje y el perfil QoS YA EXISTAN antes de referenciarlos.**
   `node_config.py` no crea nada nuevo, solo resuelve nombres a algo que ya
   está instalado:
   - Tipo de mensaje (`message_type: pkg/msg/Tipo`): tiene que existir ya en
     algún paquete de interfaces (`std_msgs`, `sensor_msgs`, `geometry_msgs`,
     `example_interfaces`...). Si hace falta un tipo que no existe, hay una
     decisión de diseño previa: JSON dentro de `std_msgs/String` (el patrón
     que ya usan `feedback` y `to_scene_msg`/`from_scene_msg`) o compilar un
     `.msg` propio (paquete `ament_cmake` aparte, `rosidl_generate_interfaces`)
     — el YAML no decide esto por ti.
   - Perfil QoS por nombre (`qos: MI_PERFIL`): tiene que estar ya en
     `_QOS_BY_NAME` (`ros2_kit/node_config.py`), definido en
     `ros2_kit/qos.py` con su motivo documentado (ver el resto de perfiles
     ahí). Un `qos:` numérico (profundidad simple) no necesita este paso.
4. **Declarar la dependencia en el `package.xml` del paquete del nodo** si
   el tipo de mensaje viene de un paquete ROS2 nuevo para ese nodo (p. ej.
   añadir `<depend>geometry_msgs</depend>` si empiezas a usar `Pose` en un
   nodo que antes no lo necesitaba). No rompe nada en local si se olvida
   (el import ya está instalado por el sistema), pero es lo que usa
   `rosdep`/CI para saber qué instalar — se detecta tarde si no se hace.
5. **`colcon build --symlink-install --packages-select <paquete>`** —
   hace falta para que el YAML se instale en `share/<paquete>/config/`,
   aunque el `.py` esté symlinkeado (symlink-install symlinka archivos que
   YA existían al build; un YAML nuevo o editado necesita este paso para
   que `package_config_path` lo encuentre).
6. **Verificar construyendo el nodo de verdad** (`rclpy.init()` + la clase
   del nodo, o `ros2 run`), no solo confiar en tests con dobles de prueba.
   Así se encontró un bug real al escribir esto: `rclpy.node.Node` ya usa
   `self._publishers` como lista interna propia (la necesita
   `destroy_node()`); guardar ahí el dict de `apply_node_config` la pisaba
   y rompía el apagado del nodo — silencioso hasta que se construye uno de
   verdad, ningún test con doble lo habría detectado.
7. **Mantener sincronizada la documentación** — la tabla de este documento
   y, si aplica, el `README.md` del paquete (`src/<paquete>/README.md`).
   Son la fuente de verdad práctica; si no se actualizan, mienten.
