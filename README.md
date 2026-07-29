# robot-control-hexagonal

Arquitectura hexagonal/DDD para el control del brazo Dobot CR5 (simulado en
CoppeliaSim, y en el futuro real), sobre ROS2 Humble.

## Lenguaje común

`Comandante -> Trajectory -> set_joints`, más un puerto de cálculo separado:

- **`RobotControllerPort`** (`shared_kernel/ports.py`) — el "Nodo Robot": ejecuta
  `set_joints(configuration)` y reporta `get_current_configuration()`. No calcula
  nada, solo obedece y reporta. Adaptadores: CoppeliaSim (real, funcional) y
  CR5 físico (pendiente — el driver oficial de Dobot es ROS1, no ROS2).
- **`KinematicsPort`** (`shared_kernel/ports.py`) — el "Nodo Controlador": calcula
  una `Trajectory` para alcanzar un objetivo cartesiano (`Pose`). Adaptadores:
  PoE/Explicit, GA/gafro, DH numérico (los tres pendientes de implementar la
  matemática real) y `naive_test` (doble de pruebas, ignora el objetivo,
  solo sirve para probar el cableado).
- **`Commander`** — no es un puerto, es la capa de aplicación: crea
  `ControlSession`s (empareja un Robot con un Controlador en un namespace ROS2
  propio, con vida acotada), les manda objetivos y escucha su feedback.

## Paquetes

```
src/
├── shared_kernel/     dominio puro (value objects, Either, Trajectory, puertos) — sin ROS2
├── robot_node/         paquete ROS2: adaptador ROS2 alrededor de RobotControllerPort
├── controller_node/    paquete ROS2: adaptador ROS2 alrededor de KinematicsPort
└── commander/          paquete ROS2: ControlSession + Commander
```

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

## Lo que falta (a propósito, documentado en el propio código)

- `PoeKinematicsAdapter`, `GaKinematicsAdapter`, `DhKinematicsAdapter`: lanzan
  `NotImplementedError` con una nota de qué hace falta exactamente.
- `Cr5RealRobotAdapter`: igual — el driver oficial de Dobot es ROS1, hace
  falta reimplementar el protocolo TCP/IP o levantar un `ros1_bridge`.
