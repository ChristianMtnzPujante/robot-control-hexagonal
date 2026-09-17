---
tags: [arquitectura, nodo]
---

# Herramientas de CoppeliaSim

Dos módulos de `commander` que orquestan CoppeliaSim como proceso externo
— no son puertos del dominio, son utilidades de infraestructura para
demos y para construir escenas por código en vez de a mano. Referencia
función por función de `coppeliasim_launcher.py` y
`coppeliasim_scene_builder.py`.

## `coppeliasim_launcher.py`

Arranca (o reutiliza) una instancia de CoppeliaSim con una escena `.ttt`
ya hecha, cargada y en play — para que `robot_node`/`controller_node`
arranquen después sin condición de carrera contra `simIK.addElementFromScene`
buscando objetos que aún no existen. Solo lo usa `two_sessions_demo.py`.

- **`ensure_coppeliasim_scene(port, settings_suffix, scene_path, timeout=90.0)`**
  — si `port` ya tiene algo escuchando, lo reutiliza tal cual (se asume
  instancia válida) y solo le manda cargar `scene_path`; si no, lanza una
  instancia nueva y espera a que responda. Para cargar la escena para la
  simulación primero (`stopSimulation`, idempotente si ya lo estaba —
  mismo patrón que `CoppeliaSimRobotAdapter._load_scene_and_play`), hace
  `loadScene` y vuelve a arrancar.
- **`_port_open(port, host="localhost")`** — sondea con un socket TCP con
  timeout de 0.5s; es el mecanismo tanto para decidir si reutilizar una
  instancia como para esperar a que una nueva termine de arrancar.
- **`_wait_for_port(port, timeout)`** — sondea `_port_open` cada segundo
  hasta `timeout`; usado tras lanzar una instancia nueva.
- **`_launch(port, settings_suffix)`** — lanza `coppeliaSim.sh` con
  `-GzmqRemoteApi.rpcPort=<port>` (confirmado que fuerza el puerto ZMQ) y
  `COPPELIASIM_USER_SETTINGS_FOLDER_SUFFIX=<settings_suffix>` en el
  entorno. Este suffix es imprescindible para correr dos instancias a la
  vez: sin carpetas de settings distintas, ambas compiten por el mismo
  `usrset.txt` y ninguna termina de arrancar. Siempre con GUI — el modo
  headless (`-h`) no es fiable en este entorno (el proceso termina solo a
  los pocos segundos).

## `coppeliasim_scene_builder.py`

Construye una escena de CoppeliaSim **por código** a partir de un URDF
real y una `Scene` de dominio, en vez de depender de un `.ttt` hecho a
mano como `cr5_base.ttt` — pieza del Bloque 9 (generalización a robot
arbitrario). Hace lo inverso de
`perception_node/adapters/static_perception_adapter.py`: en vez de
PRODUCIR una `Scene` a partir del mundo, CONSUME una `Scene` para
construir un mundo. Deliberadamente fuera de `geometry_kernel`: `Scene`
es dominio puro y no debe saber qué es CoppeliaSim.

- **`ensure_coppeliasim_running(port, settings_suffix, timeout=90.0)`** —
  a diferencia de `ensure_coppeliasim_scene`, NO toca qué escena hay
  cargada ni el estado de simulación: solo garantiza que hay una instancia
  escuchando en `port` (la lanza si hace falta). El punto de partida es la
  escena en blanco (o lo que ya hubiera, si se reutiliza instancia).
- **`build_cr5_scene(port, initial_configuration, scene)`** — envoltorio
  de `build_scene` con las constantes del CR5 (`_CR5_URDF_PATH`,
  `_CR5_JOINT_NAMES`, `_CR5_TIP_NAME`...), conservado por compatibilidad
  con los demos que ya lo llaman así.
- **`build_scene(port, urdf_path, urdf_package_prefix, joint_names, tip_name, root_link_visual_alias, initial_configuration, scene)`**
  — el mecanismo real, ya genérico para cualquier URDF (verificado en vivo
  importando también un Franka Panda de 7 GDL junto al CR5, ver
  [[CR5 vs Panda (Generalización)]]). Importa el robot vía
  `simURDF.importFile`, lo fuerza a modo cinemático/estático (ver más
  abajo), lo coloca en `initial_configuration`, añade un marcador visual
  por cada `SphereObstacle` de `scene.obstacles`, y arranca la simulación.
  Devuelve un `CoppeliaSimRobotAdapter` ya conectado, listo como
  `RobotConnectorPort` sin volver a resolver handles. Sin marcador de
  goal — el goal no es parte de `Scene`, se marca aparte con
  `.mark_goal(...)` sobre el adaptador devuelto.

  > [!bug] `simURDF.import` deja el modelo en modo DINÁMICO (física real)
  > Descubierto en vivo, en dos capas: (1) los joints quedan en
  > `jointmode_dynamic`, por lo que `set_joints` (pensado para joints
  > cinemáticos) no se sostiene entre waypoints; (2) aunque se fuerce el
  > joint a cinemático, los shapes "respondable" siguen dinámicos y el
  > motor de físicas los mueve de forma independiente del árbol
  > cinemático — el ángulo del joint queda correcto, pero la propagación
  > visual se rompe. `cr5_base.ttt` ya traía esto resuelto a mano;
  > `build_scene` tiene que forzarlo tras importar, a los dos niveles
  > (`setModelProperty` + `setJointMode` por joint).

- **`_clear_previous_build(sim, root_link_visual_alias)`** — idempotente:
  si la misma instancia de CoppeliaSim ya tenía una escena construida por
  esta función, borra el robot anterior (`sim.removeModel`, borra todo el
  árbol de una vez) y los marcadores de obstáculo/objetivo/trail previos
  antes de reimportar — sin esto, cada reutilización acumularía un robot
  duplicado con handles ambiguos (`joint1`, `joint1_2`, ...). Solo mira
  objetos de primer nivel (sin padre); todo lo demás cuelga de alguno como
  hijo y se borra con él.
- **`save_scene(port, path)`** — persiste la escena actual (robot +
  marcadores) como `.ttt`, recargable después con
  `ensure_coppeliasim_scene`. El punto de partida sigue siendo reconstruir
  desde la descripción (URDF + `Scene`), no depender de este archivo
  guardado.

## Nota sobre el marco de referencia (hallazgo ya resuelto)

Importar el CR5 desde su URDF real, sin la opción "centrar modelo" de
`simURDF` (bit 32 de `_IMPORT_OPTIONS`), deja `base_link_respondable`
exactamente en el origen del mundo con orientación identidad —
verificado que `Link6_visual` en configuración cero coincide con
`PoeKinematicsAdapter.forward_kinematics` al micrómetro. Esto elimina un
hallazgo de marco sin resolver que documentaba `two_sessions_demo.py`
(marco interno de `KinematicsPort` frente al marco mundo de CoppeliaSim)
y que antes obligaba a calibrar una transformación a mano: con este
constructor, marco interno == marco mundo por construcción.

## Ver también

- [[Commander (referencia de código)]]
- [[Commander y ControlSession]]
- [[CoppeliaSimRobotAdapter]]
- [[CR5 vs Panda (Generalización)]]
- [[Estado del Roadmap]]
