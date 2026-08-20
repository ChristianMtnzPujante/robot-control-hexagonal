# robot-control-hexagonal

Arquitectura hexagonal/DDD para el control del brazo Dobot CR5 (simulado en
CoppeliaSim, y en el futuro real), sobre ROS2 Humble.

## Lenguaje común

`Comandante -> Trajectory -> set_joints`, más los puertos de cálculo separados:

- **`RobotControllerPort`** (`shared_kernel/ports.py`) — el "Nodo Robot": ejecuta
  `set_joints(configuration)` y reporta `get_current_configuration()`. No calcula
  nada, solo obedece y reporta. Adaptadores: CoppeliaSim (real, funcional) y
  CR5 físico (pendiente — el driver oficial de Dobot es ROS1, no ROS2).
- **`KinematicsPort`** (`shared_kernel/ports.py`) — el "Nodo Controlador": cinemática
  inversa pura, de un objetivo cartesiano (`Pose`) a una `Trajectory` alcanzable,
  sin conocer la escena ni evitar nada. Adaptadores: PoE/Explicit, GA/gafro, DH
  numérico (los tres pendientes de implementar la matemática real) y
  `naive_test`/`straight_line` (dobles de pruebas, solo sirven para probar el
  cableado).
- **`PlanningPort`** (`shared_kernel/ports.py`) — como `KinematicsPort`, pero
  recibe también una `Scene` (obstáculos, planos) y debe evitarlos. Pensado
  para CHOMP/RRT; sin ningún adaptador todavía.
- **`PlannerSelectionPort`** (`shared_kernel/ports.py`) — elige qué estrategia
  de planificación usar según el estado de la `Scene`. Sin ningún adaptador
  todavía.
- **`Scene`** (`geometry_kernel/scene.py`) — no es un puerto, es el agregado que
  representa lo que el sistema sabe de la escena (planos, obstáculos, objetos
  detectados). La producirá percepción (pendiente) y la consumen
  `PlanningPort`/`PlannerSelectionPort`. Hoy solo existe `Scene.empty()`.
- **`Commander`** — no es un puerto, es la capa de aplicación: crea
  `ControlSession`s (empareja un Robot con un Controlador en un namespace ROS2
  propio, con vida acotada), les manda objetivos y escucha su feedback.

## Paquetes

```
src/
├── geometry_kernel/    primitivas geométricas puras (Pose, Point, Plane, Scene) — sin ROS2, sin depender de shared_kernel
├── shared_kernel/     dominio de ejecución/cinemática (value objects, Either, Trajectory, puertos) — sin ROS2, depende de geometry_kernel
├── ros2_kit/           infraestructura ROS2 compartida: mensajes <-> dominio, ciclo de vida de nodos
├── ros1_kit/           BOCETO sin usar: construir/gestionar un puente ros1_bridge programáticamente
├── robot_node/         paquete ROS2: adaptador ROS2 alrededor de RobotControllerPort
├── controller_node/    paquete ROS2: adaptador ROS2 alrededor de KinematicsPort/PlanningPort
└── commander/          paquete ROS2: ControlSession + Commander
```

`geometry_kernel` es el nivel más bajo: primitivas puramente geométricas
(`Pose`, `Point`, `Plane`, `SphereObstacle`, `Scene`), sin depender de nada —
ni siquiera de `shared_kernel`; es al revés, `shared_kernel` depende de
`geometry_kernel` y reexporta `Pose`/`Scene` para que el resto de paquetes
siga haciendo `from shared_kernel import ...` sin enterarse del cambio. La
razón de separarlos: `Scene` la va a necesitar tanto el contexto de
ejecución/cinemática como el futuro contexto de percepción, y percepción no
necesita saber nada de `JointConfiguration` ni de `Trajectory`.

`ros2_kit` existe para que `robot_node`/`controller_node`/`commander` no
repitan la conversión de mensajes ni el ciclo de vida de `rclpy` — pero
`shared_kernel` sigue sin depender de él ni de ROS2 en absoluto. `ros1_kit`
es un boceto deliberadamente sin conectar a nada todavía (piensa en el día
que haga falta un puente `ros1_bridge` hacia el driver oficial del CR5,
que es ROS1) — mismo principio: cuando exista, ninguno de los dos
tocará `shared_kernel`.

Los puertos son `typing.Protocol`, no `ABC` — un adaptador no necesita heredar
de nada, solo implementar los métodos exigidos (duck typing, verificado
estáticamente por mypy). Esto es lo que hace el sistema adaptable: añadir una
estrategia de cinemática nueva, o un robot nuevo, es añadir una clase, sin
tocar el dominio ni el Commander.

## Cómo se hablan los nodos

Cada `ControlSession` vive en su propio namespace (`/session_<nombre>`):

```
Commander --publica--> <ns>/goal (geometry_msgs/Pose) --> controller_node
controller_node --publica--> <ns>/joint_command (sensor_msgs/JointState) --> robot_node
robot_node --publica--> <ns>/joint_states (sensor_msgs/JointState) --> controller_node (feedback)
controller_node --publica--> <ns>/feedback (std_msgs/String) --> Commander
```

No hace falta ningún mensaje personalizado — todo son tipos estándar de ROS2
(`sensor_msgs`, `geometry_msgs`, `std_msgs`), así que no hay que compilar
`.msg` propios.

## Compilar y probar

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/robot-control-hexagonal
colcon build --symlink-install
source install/setup.bash

# CoppeliaSim debe estar abierto con la escena cr5_base.ttt cargada y
# la simulación EN PLAY antes de correr esto (el adaptador de CoppeliaSim
# necesita que sim.setJointPosition/getJointPosition respondan).

ros2 run commander commander_demo
```

Con eso deberías ver, en otra terminal, el robot en CoppeliaSim moviéndose
+0.1 rad en cada articulación (el doble de pruebas `naive_test`), y en la
terminal del `commander_demo` los mensajes de `feedback` según van llegando.

## Demo: dos sesiones independientes, dos cálculos de cinemática distintos

```bash
source install/setup.bash
ros2 run commander commander_demo_two_sessions
```

Arranca **dos** `ControlSession` a la vez, cada una contra su propia
instancia de CoppeliaSim (puertos 23000/23001) y con una estrategia de
cinemática distinta: una resuelve con `PoeKinematicsAdapter` (matemática
propia), la otra delega en `simIK` vía `CoppeliaSimIkKinematicsAdapter`. Si
no hay ya CoppeliaSim abierto en esos puertos, la demo lo lanza sola (dos
ventanas nuevas) y carga la escena correspondiente
(`cr5_base.ttt`/`cr5_base_pruebas.ttt`); si ya lo hay, lo reutiliza. Sirve
como ejemplo de que crear/conectar un robot y elegir su cálculo de
cinemática es cuestión de los parámetros de `Commander.create_session`, no
de tocar código — ver `commander/two_sessions_demo.py`.

El modo headless de CoppeliaSim no es fiable en esta clase de entorno (el
proceso puede cerrarse solo a los pocos segundos) — la demo siempre abre
ventana con GUI.

## Lo que falta (a propósito, documentado en el propio código)

- `PoeKinematicsAdapter`, `GaKinematicsAdapter`, `DhKinematicsAdapter`: lanzan
  `NotImplementedError` con una nota de qué hace falta exactamente.
- `Cr5RealRobotAdapter`: igual — el driver oficial de Dobot es ROS1, hace
  falta reimplementar el protocolo TCP/IP o levantar un `ros1_bridge`.
- `PlanningPort`, `PlannerSelectionPort`: puertos definidos, sin ningún
  adaptador todavía (CHOMP/RRT y la selección entre ellos — ver
  `ROADMAP.md`, Bloques 4 y 5).
- `Scene`: agregado definido, sin ningún productor real todavía (percepción
  — ver `ROADMAP.md`, Bloque 3). Solo existe `Scene.empty()`.

Ver `ROADMAP.md` para la hoja de ruta completa hacia el backend de
"descripción en lenguaje natural → nodo robótico validado".
