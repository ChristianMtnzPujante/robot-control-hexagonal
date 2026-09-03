# Contrato de nodos ROS2

Este documento define los tipos de proceso ROS2 que componen el sistema,
qué responsabilidad tiene cada uno, qué sabe y qué NO debe saber nunca, y
el contrato de comunicación (topics, tipos de mensaje, semántica) entre
ellos. Es el complemento operativo de la sección "Lenguaje común" del
`README.md` — aquí se detalla el contrato completo, no solo el resumen.
Para la referencia práctica de parámetros por nodo (valores, rangos, cómo
sobreescribirlos), ver `docs/configuracion_nodos.md` en su lugar — este
documento es sobre el CONTRATO (topics, responsabilidades), no sobre cómo
lanzar/configurar un nodo concreto.

Como el resto del repo, distingue explícitamente **estado actual
(implementado)** de **diseño objetivo (pendiente, ver `ROADMAP.md`)** — no
mezclar ambos es la razón de ser de este documento: evita que se asuma que
algo existe solo porque "tendría sentido que existiera".

## 1. Tipos de proceso

Hay tres tipos de proceso ROS2, más un orquestador que no es un nodo ROS2
en sí mismo:

| Proceso | Paquete | Adapta el puerto | Es un nodo ROS2 |
|---|---|---|---|
| `Commander` | `commander` | (ninguno — capa de aplicación) | Sí (`commander`) |
| `ControlSession` | `commander` | (ninguno — orquestador de procesos) | No — vive dentro del proceso `Commander`, lanza a los otros dos como `subprocess.Popen` |
| `robot_node` | `robot_node` | `RobotControllerPort` | Sí (`robot_node`) |
| `controller_node` | `controller_node` | `KinematicsPort` / `PlanningPort` / `PlannerSelectionPort` | Sí (`controller_node`) |

Cada `ControlSession` lanza **un `robot_node` y un `controller_node` como
procesos del sistema operativo separados** (no hilos, no imports) — así
sim/real, o dos máquinas distintas, pueden ejecutarse por separado sin
tocar código (ver `control_session.py`).

### 1.1 `Commander`

**Responsabilidad:** crear `ControlSession`s, mandarles objetivos
cartesianos, escuchar su `feedback`, y supervisar que los procesos sigan
vivos.

**Qué SABE:** namespaces de sesión (`/session_<nombre>`), el lenguaje común
`Pose -> feedback`, y (diseño objetivo, no implementado) el estado de vida
de los procesos que lanzó.

**Qué NO debe saber nunca** (invariante de diseño, ya en el docstring de
`commander_node.py`): que existen CoppeliaSim, PoE, GA/gafro, DH, o el CR5
físico. Tampoco debe saber qué estrategias de cinemática/planificación
existen ni cuál está activa en un momento dado — eso es responsabilidad
interna de `controller_node` (ver §1.3 y §4).

**Entrada:** invocaciones directas de aplicación (`create_session`,
`send_goal`, `close_session` — hoy llamadas desde `main()`, mañana desde
la API de tools del Bloque 6 del ROADMAP).

**Salida:** un `Pose` publicado en `<ns>/goal` por sesión; logs de
`feedback` recibido.

**Estado actual:** `create_session` recibe una `controller_strategy` que
se pasa una única vez a `controller_node` al lanzarlo — es un valor
*inicial*, no una decisión que `Commander` pueda revisar después.
`ControlSession.stop()` existe pero nada llama a relanzar un proceso
muerto — no hay supervisión real todavía.

### 1.2 `robot_node`

**Responsabilidad única:** traducir entre mensajes ROS2 y el dominio de
ejecución, y delegar en un adaptador concreto de `RobotControllerPort`.
Ejecuta `set_joints(configuration)` cuando le llega un `joint_command`, y
publica `get_current_configuration()` periódicamente como `joint_states`.

**Qué SABE:** la forma física del robot que controla (`joint_names`) y el
driver concreto elegido por parámetro (`robot_target`: `"simulado"` →
`CoppeliaSimRobotAdapter`, `"real"` → `Cr5RealRobotAdapter`, pendiente).

**Qué NO sabe ni decide:** nada de cinemática, nada de objetivos
cartesianos, nada de estrategias. No sabe si el `joint_command` que recibe
viene de `naive_test`, `coppeliasim_ik` o cualquier otro adaptador de
`controller_node` — para `robot_node`, todos son iguales: una secuencia de
`JointConfiguration`.

**No forma parte de `RobotControllerPort`, pero este adaptador concreto lo
ofrece igualmente** (decoración puramente visual, opcional): `mark_goal`
(deja un dummy en el punto objetivo) y el trail de waypoints si se le da
`tip_name`. `robot_node` los usa solo si el adaptador los expone
(`getattr(..., None)`), sin acoplarse a que existan.

### 1.3 `controller_node`

**Responsabilidad:** dado un objetivo cartesiano (`Pose`) y la
configuración actual del robot, calcular una `Trajectory` alcanzable y
enviarla como secuencia de `joint_command`, reportando progreso por
`feedback`.

**Diseño objetivo (ver conversación de diseño y ROADMAP Bloque 5):**
`controller_node` debe poseer **toda** la responsabilidad de "cómo llegar
del objetivo a una trayectoria", incluida la decisión de **qué estrategia
usar y cuándo cambiarla**. Eso significa que la estrategia deja de ser un
valor fijo pasado una vez al arrancar, y pasa a estar gestionada
internamente por un `PlannerSelectionPort` (puerto de dominio, en
`shared_kernel`, testeable sin ROS2) que decide según el estado de la
`Scene`. `Commander` no participa en esa decisión — como mucho, le da una
estrategia *inicial* de arranque.

**Estado actual (implementado):** la estrategia se fija una única vez, al
arrancar el proceso, vía el parámetro ROS2 `strategy` (`declare_parameter`
+ `_build_adapter(strategy)` en `__init__`). No hay ningún mecanismo —ni
topic, ni servicio, ni callback de parámetros— para cambiarla después de
creado el nodo. `PlannerSelectionPort` está definido en
`shared_kernel/ports.py` pero **ningún adaptador lo implementa todavía**.

**Qué SABE:** las estrategias disponibles (`poe`, `ga`, `dh`, `naive_test`,
`straight_line`, `coppeliasim_ik`, y en el futuro `chomp`/`rrt` vía
`PlanningPort`) y, en el diseño objetivo, cuándo conmutar entre ellas.

**Qué NO sabe:** cómo se ejecuta físicamente un `joint_command` — eso es
responsabilidad exclusiva de `robot_node`. `controller_node` nunca habla
directamente con CoppeliaSim salvo que el propio adaptador de cinemática
elegido lo requiera (caso de `coppeliasim_ik`, que rompe deliberadamente
la promesa de agnosticismo de plataforma de `KinematicsPort` a cambio de
tener IK físicamente correcta ya — ver docstring del propio adaptador).

## 2. Namespacing

Cada `ControlSession` vive en su propio namespace ROS2:

```
/session_<nombre>/goal
/session_<nombre>/joint_command
/session_<nombre>/joint_states
/session_<nombre>/feedback
```

`robot_node` y `controller_node` de una misma sesión se "conocen" única y
exclusivamente por compartir ese namespace — ninguno de los dos tiene
referencia directa al otro (ver docstring de `control_session.py`).

## 3. Contrato de topics (estado actual, implementado)

Todos los tipos son estándar de ROS2 — no hay ningún `.msg` propio que
compilar.

| Topic | Tipo | Publica | Suscribe | QoS | Semántica del payload |
|---|---|---|---|---|---|
| `<ns>/goal` | `geometry_msgs/Pose` | `Commander` | `controller_node` | `GOAL_QOS` (RELIABLE + TRANSIENT_LOCAL, depth 1) | Objetivo cartesiano a alcanzar. `GOAL_QOS` existe para que el orden de arranque entre procesos no importe: retiene el último mensaje para quien se suscriba después (ver `ros2_kit/qos.py`). |
| `<ns>/joint_command` | `sensor_msgs/JointState` | `controller_node` | `robot_node` | default (volatile, depth 10) | Un waypoint de la trayectoria calculada, uno por ciclo de `_advance_trajectory` (periodo `waypoint_period_seconds`). |
| `<ns>/joint_states` | `sensor_msgs/JointState` | `robot_node` | `controller_node` | default (volatile, depth 10) | Configuración de articulaciones reportada por el robot (sim o real), a `state_publish_period_seconds`. También es lo que `controller_node` usa como `current_configuration` para calcular la siguiente trayectoria. |
| `<ns>/feedback` | `std_msgs/String` (JSON) | `controller_node` | `Commander` | default (volatile, depth 10) | Estados: `esperando_estado_robot`, `calculando`, `trayectoria_calculada` (+ `waypoints`), `waypoint_enviado` (+ `restantes`), `completado`. |

Nótese la asimetría: `goal` también llega a `robot_node` (para el
`mark_goal` cosmético opcional descrito en §1.2), pero eso no forma parte
del contrato funcional — `robot_node` seguiría funcionando
correctamente si nunca recibiera `goal`.

## 4. Contrato de topics (diseño objetivo, pendiente)

Para que `controller_node` pueda conmutar de estrategia en caliente sin
que `Commander` decida por él (§1.3), hace falta un canal nuevo. Boceto,
mismo patrón que `goal`:

| Topic | Tipo | Publica | Suscribe | Semántica propuesta |
|---|---|---|---|---|
| `<ns>/scene` (o servicio) | mensaje a definir, deriva de `Scene` | `Commander` (ensamblado, ver abajo) | `controller_node` | Estado actual de la escena, ya ensamblada — lo que consume `PlannerSelectionPort.select(scene)` internamente. |
| `<ns>/strategy_changed` | `std_msgs/String` (JSON) | `controller_node` | `Commander` (informativo) | Notificación de que `controller_node` cambió de estrategia por su cuenta — parte del `feedback`, no una orden. |

**Revisado (conversación 02/09):** el productor de `<ns>/scene` NO es un
único perceptor hablando directo con `controller_node` — es `Commander`.
Puede haber varios perceptores (cada uno detectando piezas distintas:
obstáculos, objetos...), cada uno con vida propia FUERA de cualquier
`ControlSession` (spike de ciclo de vida ya resuelto, Vikunja #89).
`Commander` los escucha, ensambla una `Scene` completa vía
`Scene.merge` (`geometry_kernel`, ver ROADMAP.md Bloque 3), y reenvía el
resultado a `<ns>/scene` de cada sesión que lo necesite — mismo rol que
ya tiene al publicar `<ns>/goal` por sesión, extendido a un canal más.
`controller_node` cachea la última `Scene` recibida, igual que ya hace
con `joint_states`→`current_configuration`. Esto NO viola el invariante
de §1.1 (`Commander` no debe saber de estrategias/backends): ensamblar
`Scene` es fusión de datos (dicts por clave), no una decisión de
planificación — sigue sin saber qué es `PlannerSelectionPort` ni qué
estrategia hay activa.

Sigue sin decidir el formato concreto del mensaje (JSON en
`std_msgs/String`, como `feedback`, o un `.msg` propio) — sin eso no
puede escribirse el `node.py` ROS2 real de `perception_node` ni el lado
de `Commander` que ensambla. Mientras tanto, `FilePerceptionAdapter`
(`perception_node/adapters/`) permite probar el `PerceptionPort` y
`Scene.merge` sin ningún wiring ROS2 todavía.

Explícitamente **no** se propone un topic `<ns>/set_strategy` publicado
por `Commander` — eso reintroduciría la decisión de estrategia en la capa
de aplicación, violando la invariante de §1.1. Si en algún momento hace
falta forzar una estrategia desde fuera (p. ej. depuración manual), debe
tratarse como una anulación explícita y explícitamente registrada, no como
el mecanismo normal de conmutación.

Pendiente también de diseñar: qué pasa con los `_pending_waypoints` en
curso cuando cambia la estrategia — ¿se descartan, se recalculan desde la
posición actual, o se deja terminar la trayectoria activa y el cambio
aplica solo al siguiente ciclo de planificación? Esto es lógica de
dominio (parte de lo que debe decidir `PlannerSelectionPort` o quien lo
orqueste dentro de `controller_node`), no un detalle de transporte.

## 5. Supervisión de procesos (diseño objetivo, pendiente)

Hoy `ControlSession.start()` lanza los dos procesos y no vuelve a
mirarlos; `stop()` existe pero nadie lo llama automáticamente ante un
fallo. Importante: si `controller_node` o `robot_node` mueren, **no llega
ningún `feedback` avisándolo** — simplemente dejan de publicar. Detectar
esto requiere vigilar el proceso en sí (`Popen.poll()`), no solo escuchar
`feedback`. Queda como responsabilidad de `Commander`/`ControlSession`
(es infraestructura de proceso, no lógica de dominio), añadir esa
vigilancia y la política de relanzamiento.

## 6. Relación con los puertos de dominio (`shared_kernel/ports.py`)

| Puerto (Protocol) | Nodo que lo adapta | Adaptadores existentes | Adaptadores pendientes |
|---|---|---|---|
| `RobotControllerPort` | `robot_node` | `CoppeliaSimRobotAdapter` | `Cr5RealRobotAdapter` |
| `KinematicsPort` | `controller_node` | `NaiveTestKinematicsAdapter`, `StraightLineKinematicsAdapter`, `CoppeliaSimIkKinematicsAdapter` | `PoeKinematicsAdapter`, `GaKinematicsAdapter`, `DhKinematicsAdapter` |
| `PlanningPort` | `controller_node` (futuro) | — | CHOMP, RRT (Bloque 4) |
| `PlannerSelectionPort` | `controller_node` (futuro, interno) | — | Bloque 5 |
| `PerceptionPort` | `perception_node` (sin `node.py` ROS2 todavía — ver §4) | `StaticPerceptionAdapter`, `PseudoPerceptionAdapter`, `FilePerceptionAdapter` | `CoppeliaSimPerceptionAdapter` (ground truth, bloqueado en cómo leer el radio de una esfera vía la API ZMQ — ver ROADMAP.md Bloque 3) |

Los puertos son `typing.Protocol`: un nodo no "hereda" nada, un adaptador
nuevo solo necesita implementar los métodos exigidos. Esto es lo que
permite que este documento defina *responsabilidades y contratos de
comunicación* sin necesitar tocar código cada vez que se añade un
adaptador — el contrato entre nodos no cambia, solo qué hay detrás de
`self._kinematics` o `self._robot_controller`.
