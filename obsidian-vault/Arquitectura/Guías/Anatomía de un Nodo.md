---
tags: [arquitectura, guia]
---

# Anatomía de un Nodo

Configurar un nodo en este repositorio siempre se reparte en las mismas
tres piezas — una es dato, dos son código. Esta nota traza esa frontera y
sigue un mensaje real de principio a fin, para no confundir lo que hay que
escribir con lo que ya viene resuelto. Necesaria para saber DÓNDE declarar
un parámetro ROS2 nuevo que un adaptador nuevo probablemente necesite (p.
ej. `cr5_host` en `robot_node.yaml` para [[Cr5RealRobotAdapter]]) — ver
también [[Conectar un Robot Nuevo]].

## 1 · DATO — el YAML declara qué existe

Un parámetro, un publisher, una suscripción o un timer se escriben una vez
en `config/<paquete>.yaml` — nunca como una llamada suelta a
`declare_parameter`/`create_publisher` dentro de `__init__`. El YAML solo
dice **qué** existe: nombre, tipo, QoS, valor por defecto. Nunca dice
**qué hace** — esa es la frontera con la siguiente pieza.

```yaml
parameters:
  waypoint_period_seconds:
    type: double
    default: 0.5
    range: {min: 0.05, max: 5.0}   # rclpy rechaza fuera de rango, en runtime

subscriptions:
  - topic: goal
    message_type: geometry_msgs/msg/Pose
    qos: GOAL_QOS
    callback: _on_goal            # solo el nombre — el cuerpo vive en node.py
```

- `range` se traduce a un `ParameterDescriptor` real de `rclpy` — el
  rechazo de un valor fuera de rango lo hace ROS2, no una validación
  propia.
- `message_type` es la ruta `paquete/msg/Tipo` — tiene que existir ya,
  compilado; el YAML lo resuelve, no lo crea.
- `qos` referencia un perfil *por nombre*, definido aparte en
  `ros2_kit/qos.py` con su motivo documentado — el YAML no inventa
  políticas de entrega nuevas.

## 2 · CÓDIGO — `messages.py` traduce dominio ↔ ROS2

Un mensaje ROS2 (`geometry_msgs/msg/Pose`) y el value object de dominio
(`shared_kernel.Pose`) son dos modelos distintos con el mismo nombre. Esa
traducción es la única serialización que se escribe a mano — todo lo que
pasa después camino de la red (serializar a bytes, cruzar la red,
deserializar) lo generó `rosidl` al compilar `geometry_msgs`, a partir de
`Pose.msg`.

```python
def to_pose(msg: PoseMsg) -> Pose:
    return Pose(
        x=msg.position.x, y=msg.position.y, z=msg.position.z,
        qx=msg.orientation.x, qy=msg.orientation.y,
        qz=msg.orientation.z, qw=msg.orientation.w,
    )

def to_pose_msg(pose: Pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = pose.x, pose.y, pose.z
    msg.orientation.x, msg.orientation.y = pose.qx, pose.qy
    msg.orientation.z, msg.orientation.w = pose.qz, pose.qw
    return msg
```

- Un tipo de dato nuevo en un topic ⇒ una pareja nueva `to_X` / `to_X_msg`
  aquí. No hay forma genérica de saltárselo — la traducción semántica no
  se puede inferir del `.msg`.
- Si en vez de un tipo estándar se manda JSON dentro de `std_msgs/String`
  (como `feedback` o `Scene`), se pierde toda esta generación automática
  para ese payload — la serialización pasa a ser manual también.

## 3 · CÓDIGO — los callbacks son la lógica de verdad

El YAML solo aporta el nombre (`callback: _on_goal`); `apply_node_config`
lo resuelve con `getattr(node, "_on_goal")` en el momento de suscribirse.
El método en sí — qué decide, qué publica, qué guarda — vive donde
siempre: la clase del nodo.

```python
def _on_goal(self, msg: PoseMsg) -> None:
    goal = to_pose(msg)                          # messages.py
    if self._latest_configuration is None:
        self._pending_goal = goal                # aún no hay joint_states
        self._publish_feedback("esperando_estado_robot")
        return
    self._start_trajectory(goal)
```

- El callback tiene que existir **antes** de que el YAML lo referencie —
  `apply_node_config` no lo crea, solo lo busca.
- Publicar una respuesta usa el dict que devuelve `apply_node_config` (p.
  ej. `self._topic_publishers["feedback"]`), nunca un atributo
  `self._publishers` a secas — ese nombre ya lo usa `rclpy.node.Node` por
  dentro.

## 4 · MECANISMO — ¿de dónde sale el valor de un parámetro?

El default de `parameters:` en el YAML no es siempre el valor final — se
puede pisar al arrancar con `-p nombre:=valor`. La pregunta es CUÁNDO se
decide eso: antes de que `apply_node_config` pida nada.

Orden real de resolución: `rclpy.init(args)` parsea `--ros-args -p ...`
(en C, dentro de `rcl`) → `super().__init__()` construye
`self._parameter_overrides`, ya resuelta → solo entonces
`apply_node_config` declara y consulta esa tabla.

```python
# rclpy/node.py — Node.declare_parameters (código del framework, no del repo)
if not ignore_override and name in self._parameter_overrides:
    value = self._parameter_overrides[name].value
```

- Ningún archivo YAML "final" se construye nunca — el YAML en disco no
  cambia jamás, solo aporta el default que se usa cuando NO hay override.
- La sustitución vive por completo en memoria, por proceso, cada vez que
  arranca el nodo — para verla hace falta `ros2 param get` con el nodo ya
  corriendo, no un archivo que leer.

## 5 · MECANISMO — caso real: un valor derivado (`joint_names` desde un URDF)

Cuando un valor no es literal sino que depende de otra fuente (aquí, un
`.urdf` real), la pregunta es DÓNDE calcularlo. Se probaron las dos
formas — la comparación es la lección.

**Descartado — dentro de `robot_node`:** el nodo arranca, declara
`joint_names` con el literal del YAML, lee `urdf_path` y deriva el valor
real, y tiene que **corregir** el parámetro ya declarado vía
`set_parameters(...)` — un paso de más, y un parámetro que existe
brevemente con un valor incorrecto.

**Solución — en `ControlSession`, antes de lanzar:** `ControlSession`
(Python puro, sin nodo ROS2 todavía) deriva `joint_names` desde el URDF y
lo pasa ya resuelto como `-p joint_names:=[...]` — `robot_node`
simplemente lo declara, sin corrección posterior.

```python
# commander/control_session.py
def _resolve_joint_names(self, literal_joint_names, urdf_path, base_link, tip_link):
    if not urdf_path:
        return literal_joint_names          # comportamiento de siempre
    description = parse_urdf_file(urdf_path, base_link, tip_link)
    return [joint.name for joint in description.joints]
```

Misma derivación, un nivel más arriba: `urdf_path` deja de ser un
parámetro ROS2 de `robot_node` (overrideable por `-p`, solo resoluble una
vez el `Node` ya existe) y pasa a ser un argumento de Python de
`ControlSession`, resuelto antes de que exista ningún proceso ROS2 que
lanzar.

## Antes de dar el nodo por terminado

YAML y callback son el 80% del trabajo, pero no el 100% — estas cosas no
tienen su propio hueco en `node_config.py` y son las que más se olvidan.

**Antes de compilar**
- Tipo y QoS ya existen. `message_type` y `qos` se resuelven, no se
  crean — si hace falta un tipo o perfil nuevo, es una decisión aparte
  primero.
- Dependencia en `package.xml` si el mensaje viene de un paquete ROS2
  nuevo para ese nodo.

**Al compilar**
- `colcon build --symlink-install --packages-select <paquete>` — un YAML
  nuevo no se instala solo porque el `.py` esté symlinkeado.

**Después de compilar**
- Construir el nodo de verdad (`rclpy.init()` real, no solo tests con
  dobles) — así se encontró el choque de `self._publishers` con `rclpy`.
- Actualizar la documentación — la tabla de `docs/configuracion_nodos.md`
  y el `README.md` del paquete, o dejan de ser la fuente de verdad (y,
  desde 04/09, también la vault — ver
  [[Cómo usar este vault (Obsidian)]]).

## Ver también

- [[Conectar un Robot Nuevo]]
- [[Puertos y Adaptadores]]
- [[Commander y ControlSession]]
- [[CR5 vs Panda (Generalización)]] — puesta a prueba real de este mecanismo contra un robot de 7 GDL
