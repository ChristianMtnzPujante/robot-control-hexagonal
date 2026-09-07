---
tags: [arquitectura]
---

# Commander y ControlSession

`Commander` no es un puerto, es la capa de aplicación: crea `ControlSession`s,
les manda objetivos, escucha su feedback, y (desde Bloque 3) ensambla la
`Scene` completa a partir de lo que reporten uno o más perceptores — ver
[[Scene y Percepción]].

## ControlSession — empareja un robot con un controlador

Cada `ControlSession` vive en su propio namespace ROS2 (`/session_<nombre>`)
y lanza `robot_node`/`controller_node` como **procesos reales**
(`subprocess.Popen`, no hilos) — así se puede lanzar simulación/real por
separado, incluso en máquinas distintas si algún día hace falta (ver Bloque 8
en [[Estado del Roadmap]]).

El "cableado" entre ambos nodos es, literalmente, compartir namespace:

```
Commander --publica--> <ns>/goal (geometry_msgs/Pose) --> controller_node
controller_node --publica--> <ns>/joint_command (sensor_msgs/JointState) --> robot_node
robot_node --publica--> <ns>/joint_states (sensor_msgs/JointState) --> controller_node (feedback)
controller_node --publica--> <ns>/feedback (std_msgs/String) --> Commander
```

Todo tipos estándar de ROS2 (`sensor_msgs`, `geometry_msgs`, `std_msgs`) —
no hace falta compilar ningún `.msg` propio.

### `joint_names` puede venir de un URDF

`ControlSession._resolve_joint_names` acepta o bien una lista literal, o
(dando `urdf_path` + `base_link` + `tip_link`) deriva el orden real de
articulaciones vía `urdf_kit.parse_urdf_file`, en Python puro, **antes** de
lanzar ningún proceso — así `robot_node`/`controller_node` reciben
`joint_names` ya resuelto por `-p`, sin tener que derivarlo ellos mismos.
Parte del trabajo de generalización del Bloque 9.

> [!bug] Corregido y verificado (07/09, `e009321`) — proceso zombie en `stop()`
> `ros2 run` no hace `exec()` sobre el nodo real: el PID que devuelve
> `Popen(["ros2","run",...])` es el del *lanzador*, no el del nodo `rclpy`.
> Terminar solo ese PID mataba el lanzador pero dejaba el nodo real
> huérfano (reparentado a `init`), sin recibir nunca la señal — encontrado
> tras acumular media docena de nodos zombie en una sola sesión de trabajo,
> dos de ellos con **conexión TCP abierta al CR5 físico**. Fix: cada
> proceso arranca con `start_new_session=True` (líder de su propio grupo) y
> `stop()` señala el grupo entero vía `os.killpg` — el grupo sí incluye al
> hijo real aunque quede reparentado, la pertenencia a un grupo no cambia
> al reparentar. Verificado en vivo: sesiones reales sucesivas contra el
> CR5 físico sin dejar ningún proceso residual.
>
> Segundo hallazgo el mismo día, relacionado: el deadline fijo de la
> sesión (pensado para el ritmo casi instantáneo de CoppeliaSim) podía
> matar `robot_node` real ANTES de que terminara de procesar su cola de
> `joint_command` — sin ningún error visible, los mensajes sin procesar
> simplemente desaparecen con el proceso. Los scripts de demo contra el
> robot real ahora esperan a que `RobotMode()` vuelva a "inactivo" antes
> de cerrar la sesión.

## Ver también

- [[Puertos y Adaptadores]]
- [[Scene y Percepción]]
- [[Arquitectura Hexagonal]]
- [[Decisiones de Diseño Clave]]
