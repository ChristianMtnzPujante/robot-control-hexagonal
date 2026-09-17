---
tags: [pruebas, arquitectura]
---

# Scripts de demostración

Catálogo de los scripts ejecutables (`ros2 run <paquete> <script>`) de
`commander/` y `robot_node/` — no son módulos de librería, son flujos
lineales que validan una pieza concreta de la arquitectura contra
CoppeliaSim o contra el CR5 físico. Esta nota es el índice de qué script
corresponde a qué prueba/hallazgo ya analizado en otra nota — el análisis
en sí vive en [[Evitación de Colisiones]], [[CR5 vs Panda (Generalización)]]
o el [[Diario]], no aquí.

## Evitación de obstáculos (Bloque 4, sobre CoppeliaSim)

Los cuatro comparten la misma orquestación reutilizable
(`avoid_obstacle_demo.run`) — cambiar de ejemplo es cambiar la
"descripción" (postura inicial + `Scene` + `planner_factory`), no tocar el
cableado. Resultados numéricos y hallazgos ya analizados en
[[Evitación de Colisiones]].

- **`avoid_obstacle_demo.py`** — caso base: CR5 en postura home,
  [[ObstacleAvoidingPlanningAdapter]] (evita mirando solo el tip) rodeando
  una esfera que sí invade la línea recta al objetivo. Define `run(...)`,
  la función que reutilizan los otros tres.
- **`avoid_obstacle_demo_joint2_90.py`** — mismo `run`, postura inicial
  distinta (`joint2=90°`) y otro goal/obstáculo — demuestra que cambiar de
  ejemplo no toca la orquestación, solo la descripción.
- **`avoid_obstacle_demo_whole_body.py`** — mismo `run`, pero con
  [[WholeBodyObstacleAvoidingPlanningAdapter]] vía `planner_factory`, y un
  obstáculo colocado a propósito fuera de la línea recta del tip pero
  sobre el antebrazo real — el caso donde el planificador tip-only no ve
  nada y el de cuerpo completo sí se desvía (21 vs. 41 waypoints).
- **`avoid_obstacle_demo_compare.py`** — lanza el escenario de
  `avoid_obstacle_demo_whole_body.py` DOS veces a la vez, en dos instancias
  de CoppeliaSim (puertos 23000/23001), una con cada planificador, para
  comparar el rastro de waypoints a ojo. Es el script que se ejecuta para
  reproducir en vivo la comparación de [[Evitación de Colisiones]].

## Percepción y replanificación (Bloque 3/4)

- **`file_perception_goal_demo.py`** — cableado end-to-end por ROS2 real
  (no llamadas Python directas): `perception_node` relee un fichero de
  texto y publica `/perception/scene`; `Commander.follow_perception`
  reenvía cualquier `objetivo` nuevo como `send_goal`. Escribir una línea
  "objetivo x y z" en el fichero mueve el brazo simulado sin tocar código.
  Ver [[FilePerceptionAdapter]] y [[Scene y Percepción]].
- **`perception_replan_demo.py`** — replanificación real con
  [[PseudoPerceptionAdapter]]: calcula una trayectoria sin conocer un
  obstáculo, la empieza a ejecutar, "detecta" el obstáculo a mitad de
  camino (`report_obstacle`, disparado por Python en el propio script, no
  por un sensor) y recalcula desde la configuración actual con
  [[WholeBodyObstacleAvoidingPlanningAdapter]]. Versión mínima de
  "replanificación local cuando cambia el campo de obstáculos" (Bloque 4,
  todavía pendiente en su forma general).

## Multi-sesión y generalización

- **`two_sessions_demo.py`** — dos `ControlSession` simultáneas e
  independientes (cada una con su propia instancia de CoppeliaSim, su
  propia estrategia de cinemática — PoE en una, `coppeliasim_ik` en la
  otra). Hallazgo real documentado en su propio docstring, sin resolver
  todavía: los dos `Pose` objetivo no son intercambiables entre sesiones
  porque `KinematicsPort` no fija en qué frame va `Pose` — PoE resuelve
  relativo a `base_link`, `CoppeliaSimIkKinematicsAdapter` pasa
  coordenadas de mundo tal cual a `simIK`. No es el mismo escenario que
  [[CR5 vs Panda (Generalización)]] (esa prueba generaliza a otro ROBOT,
  no a dos sesiones concurrentes), pero comparte el espíritu de "cambiar
  de robot/estrategia es cuestión de parámetros de `create_session`".

## PoE contra el CR5 físico (Bloque 0, con `Commander`/`ControlSession` reales)

- **`poe_sim_then_real_demo.py`** — primera vez que se prueba PoE de
  verdad (no el doble `naive_test`) contra el CR5 físico, en dos fases
  separadas (`--phase sim` / `--phase real`) que exigen confirmación
  humana explícita entre una y otra. El objetivo se calcula relativo a la
  pose ACTUAL real del robot (leída por el socket real-time, de solo
  lectura). Contiene el hallazgo que dio lugar a `_wait_until_robot_idle`
  en `Commander y ControlSession` — ver el "CORREGIDO" en
  [[Commander y ControlSession]] sobre el deadline fijo que cortaba la
  cola de `joint_command` a medias.
- **`poe_lift_and_wrist_demo.py`** — combina un descenso/ascenso cartesiano
  (PoE) con un giro de `joint6` sumado linealmente en espacio de
  articulaciones sobre los mismos waypoints — bypass deliberado de
  `ControlSession`/`controller_node` porque la estrategia `"poe"` del
  pipeline ROS2 solo acepta un `Pose`, no "además gira este joint". Su
  propio docstring documenta un segundo hallazgo real: el primer intento
  des-energizaba el robot inmediatamente tras mandar el último waypoint,
  cortando el movimiento a mitad de camino (mismo patrón de causa que el
  de `poe_sim_then_real_demo.py`, corregido igual: esperar a
  `RobotMode()==5` antes de actuar).
- **`real_cr5_first_session_demo.py`** — primera vez que el camino REAL
  completo de la arquitectura (`Commander → ControlSession →
  controller_node → robot_node → Cr5RealRobotAdapter`) se prueba contra
  hardware, con la estrategia `"naive_test"` (ignora el objetivo
  cartesiano, aplica un barrido sinusoidal de amplitud pequeña) a
  propósito, para no arriesgar un salto articular grande antes de que
  `Cr5RealRobotAdapter` valide límites. Ligado al hallazgo de proceso
  zombie del 07/09 (`e009321`) — ver [[Commander y ControlSession]] y
  [[2026-09-07]]. Su docstring avisa explícitamente: no des-energiza el
  robot al terminar, hay que correr `cr5_disable_demo.py` después.

## Verificar en CoppeliaSim antes de tocar hardware real

- **`cr5_go_home_sim_demo.py`** (`commander`, 08/09) — contrapartida en
  simulación de `cr5_go_home_demo.py` (más abajo): construye una escena en
  blanco con `coppeliasim_scene_builder.build_cr5_scene` (mismo importador
  de URDF que `avoid_obstacle_demo.py`), coloca el CR5 en una postura de
  partida no trivial (por defecto, no home) y anima el trayecto hasta la
  home real con `Trajectory.straight_line` en varios pasos — necesario
  porque, a diferencia de `Cr5RealRobotAdapter.set_joints` (`MovJ`, el
  robot interpola solo), `CoppeliaSimRobotAdapter.set_joints` teletransporta
  el joint al instante. Sin confirmación por teclado (no hay hardware real
  de por medio) — pensado para verificar visualmente el mismo movimiento
  antes de arriesgarlo contra el CR5 físico. Ejecutado en vivo el 08/09:
  CoppeliaSim se lanzó, la escena se construyó y el log confirmó llegar a
  `0,0,0,0,0,0` — pendiente de confirmación visual del propio usuario (sin
  captura de pantalla posible desde este entorno, XWayland bloquea el
  grab).
- **`cr5_semicircle_sim_demo.py`** (`commander`, 08/09) — traza con el tip
  una semicircunferencia antihoraria en el plano Y=constante (XZ),
  partiendo de la home; por defecto 10 puntos (9 `MovJ`/pasos), radio
  0.15m. Cada punto se resuelve con IK real (`PoeKinematicsAdapter(steps=1)`,
  encadenando cada solución como punto de partida de la siguiente) en vez
  de interpolar en espacio de articulaciones entre solo dos extremos.
  Hallazgo real por el camino: el primer diseño (arco hacia arriba) no
  convergía ni en el primer punto — ver [[PoeKinematicsAdapter]], "la home
  no puede subir, solo bajar". Diseño final: centro de la circunferencia
  justo debajo de la home, barrido de 90° a 270° (antihorario). **(08/09,
  corregido)** El `KinematicsPort` que resuelve cada punto ya no es
  `PoeKinematicsAdapter` a secas sino un
  [[SelfCollisionAwarePlanningAdapter|SelfCollisionAvoidingPlanningAdapter]]
  — ver por qué en la sección siguiente. Ejecutado en vivo contra
  CoppeliaSim tras la corrección: las 9 configuraciones completas sin
  ningún aborto, punto final exacto.
- **`cr5_circle_sim_demo.py`** (`commander`, 08/09) — a petición del
  usuario ("quiero ahora el círculo completo, evitando colisiones"):
  mismo centro/plano que el semicírculo, pero barriendo 90°→450°
  (=90°+360°, vuelta entera) en vez de 90°→270°. Por defecto 19 puntos (18
  pasos de 20°, misma resolución angular que el semicírculo). Funciona
  sin ningún caso especial porque z(theta) = centro_z + radio·sin(theta)
  nunca supera la altura de la home (sin(theta)≤1 siempre) — el círculo
  completo entero se queda dentro de la misma región alcanzable ya
  validada para el semicírculo, no hace falta volver a investigar "hacia
  dónde sí se puede". Usa `SelfCollisionAvoidingPlanningAdapter` desde el
  principio (no como corrección a posteriori). Verificado primero en
  Python puro (18 pasos, sin excepciones, cierre sobre la home con <0.1mm
  de residuo) y luego en vivo contra CoppeliaSim: círculo completo,
  vuelve exactamente al punto de partida.

  **Hallazgo real (08/09), primera vez que se probó contra el robot
  físico**: la punta cerraba el círculo con <0.05mm de error, pero
  `joint4`/`joint6` individuales quedaban a 1-2° del cero exacto — el
  usuario lo notó a simple vista ("no se ha quedado del todo recto").
  Causa: la MISMA redundancia de muñeca que
  `SelfCollisionAvoidingPlanningAdapter` explota para esquivar la
  autocolisión (varias combinaciones de `joint4`/`joint6` dan la misma
  pose del tip) deja la cadena asentada en una combinación distinta tras
  varios "nudges", no en `(0,0)`. No es un problema de calibración del
  robot. Corregido con `_snap_to_exact_start_if_needed`: si el cierre no
  coincide con la partida dentro de 0.5°, añade un último waypoint a la
  configuración de partida EXACTA (ya demostrada alcanzable, es de donde
  salió todo) — verificado de nuevo en vivo: detecta el desvío de 1.82° y
  el punto final calculado pasa a ser exactamente `0,0,0,0,0,0`.

- **`cr5_wave_sim_demo.py`** (`commander`, 14/09) — a petición del usuario
  tras el círculo ("hoy me gustaría hacer como si saludase"): el tip
  recorre un **arco** suave de lado a lado —más alto en el centro que en
  los extremos— mientras la herramienta, **inclinada hacia arriba**,
  bascula en fase con el desplazamiento. Por defecto ±0.15m, 0.05m de
  flecha de arco, ±25° de basculación, 25° de inclinación, 3 ciclos de 16
  puntos (49 waypoints). Verificado en vivo contra CoppeliaSim.

  La primera versión era una línea recta a altura constante con la
  herramienta horizontal; el usuario la corrigió en el momento ("tiene una
  pinza que sería como la mano conectada, así quedaría mejor") y de ahí
  salieron el arco y el cabeceo. Las tres piezas de geometría que lo hacen
  funcionar:

  1. **No se saluda desde la home** — está en el borde del alcance (0.933m
     de tip a hombro contra 0.9m de catálogo), así que no tiene margen
     lateral. Ver [[PoeKinematicsAdapter]], que es donde vive el número y
     dónde extiende el hallazgo del 08/09.
  2. **El cabeceo se ancla por conjugación, no por IK** —
     Ry(−90)·Rz(β)·Ry(90) = Rx(−β), así que con `joint4`=+45/`joint6`=−90
     fijos, `joint5` ES el ángulo de inclinación. Pedírselo a la IK en
     cambio llevaba a una rama contorsionada con saltos de hasta 51° entre
     waypoints. Detalle y por qué importa, en [[PoeKinematicsAdapter]] y
     [[Decisiones de Diseño Clave]].
  3. **El arco es `z = z0 − sag·s²`** (no `·s`): simétrico, centro arriba,
     ambos extremos abajo — la curva que traza una mano pivotando sobre un
     punto por debajo. La basculación es sobre el eje Y del mundo, que es
     la línea de visión del observador, así que la mano se mece en su
     plano de imagen lleve la inclinación que lleve.

  Resultado: `joint1`/`joint5`/`joint6` congelados durante todo el gesto —
  lo hacen entero `joint2`/`joint3`/`joint4`, el plano del brazo. Es
  además el recorrido cartesiano más holgado de autocolisión del repo
  (0.116m mínimo entre eslabones no adyacentes, el mismo margen que en la
  home) y cierra sobre su postura de partida con 0.01° de residuo, así que
  `_snap_to_exact_start_if_needed` ni se dispara.

## Prueba real de varios objetivos consecutivos (Bloque 0)

- **`cr5_semicircle_demo.py`** (`robot_node`, 08/09) — la misma
  semicircunferencia antihoraria que `cr5_semicircle_sim_demo.py` (ver
  más arriba, sección "Verificar en CoppeliaSim"), esta vez contra el CR5
  físico: por defecto 9 `MovJ` seguidos por la MISMA conexión de comandos,
  `cp=50`. Es, de hecho, la prueba real de "varios objetivos consecutivos
  sin cortar la conexión" que motivó toda esta serie de scripts — en vez
  de objetivos arbitrarios, una secuencia de pasos pequeños que dibuja un
  arco reconocible. Toda la IK se resuelve ANTES de mandar nada al robot;
  si algún punto no converge (o no tiene rama libre de autocolisión, ver
  abajo), se aborta sin haber tocado el robot. Aviso explícito en su
  salida: `Cr5RealRobotAdapter` valida límites absolutos por joint pero NO
  velocidad/salto entre waypoints (Bloque 0 #114, sin resolver).
  Verificado contra el arnés de servidor TCP de mentira (secuencia
  completa RobotMode→RequestControl→EnableRobot→9×MovJ→DisableRobot, sin
  fallos).

  **Hallazgo real (08/09), primera vez que se corrió contra el robot
  físico**: `DisableRobot()` falló al final con código -2.
  `GetErrorID()`=[76] = "el extremo interfiere con el cuerpo del robot" —
  autocolisión real, confirmada contra el manual oficial y la tabla de
  alarmas del fabricante. Motivó dos cosas: (1) corregir
  `cr5_disable_demo.py` para intentar `ClearError()` antes de
  `DisableRobot()` (el manual dice que en alarma ningún comando de control
  se ejecuta sin limpiarla primero); (2) el nuevo
  [[SelfCollisionAwarePlanningAdapter]] (detecta y rechaza) y su hermano
  `SelfCollisionAvoidingPlanningAdapter` (detecta e intenta esquivar
  reintentando la IK desde una muñeca desplazada — mismo archivo, ver
  [[SelfCollisionAwarePlanningAdapter]]) — este script ya usa el segundo.
  Verificado reproduciendo
  la secuencia real completa que disparó la alarma: encuentra una rama
  libre en todos los puntos, mismo destino cartesiano exacto — pendiente
  de confirmación contra el robot físico real (requiere confirmación
  interactiva, no lanzado por el asistente).
- **`cr5_circle_demo.py`** (`robot_node`, 08/09) — la versión de círculo
  completo de `cr5_circle_sim_demo.py`, contra el CR5 físico: por defecto
  18 `MovJ` seguidos por la misma conexión (el doble que el semicírculo),
  cerrando sobre el mismo punto de partida. Mismo `SelfCollisionAvoidingPlanningAdapter`
  desde el principio, y misma corrección `_snap_to_exact_start_if_needed`
  que la versión de simulación (ver más arriba): si el cierre se desvía
  más de 0.5° de la partida real, añade un 19º `MovJ` a esa configuración
  exacta. Verificado contra el arnés de servidor TCP de mentira:
  secuencia completa correcta, detecta el desvío de 1.82° y el último
  `MovJ` manda literalmente `{0,0,0,0,0,0}` — pendiente de confirmación
  contra el robot físico real (requiere confirmación interactiva, no
  lanzado por el asistente).

  **Hallazgo real (08/09), segunda vez que se corrió contra el robot
  físico**: el usuario encontró el robot parado a mitad de camino
  (`joint4≈-66°`) tras una ejecución que parecía haber terminado bien —
  sin ninguna alarma. Causa: ni este script ni `cr5_semicircle_demo.py`
  esperaban a que `RobotMode()` volviera a 5 (inactivo) antes de leer la
  posición final y des-energizar — con `cp=50` el robot sigue procesando
  la cola de `MovJ` a su propio ritmo, así que la lectura/el cierre
  podían llegar mientras seguía en marcha. Mismo patrón ya conocido de
  `commander/poe_lift_and_wrist_demo.py` y del cierre prematuro de
  [[Commander y ControlSession]] (07/09), no aplicado aquí al escribir
  estos scripts nuevos. Corregido con `_wait_until_robot_idle` en los
  dos scripts — ver [[Decisiones de Diseño Clave]].

- **`cr5_wave_demo.py`** (`robot_node`, 14/09) — el mismo saludo de
  `cr5_wave_sim_demo.py` contra el CR5 físico: por defecto 48 `MovJ`
  seguidos por la misma conexión, más del doble que el círculo completo.
  Se aparta del patrón de sus dos hermanos en una cosa: **no construye la
  trayectoria alrededor de la pose actual del robot**, porque aquí la
  postura de partida es parte del gesto (es la que da el margen de alcance
  *y* la inclinación de la mano). Va primero a ella con un único `MovJ`,
  mostrando el recorrido de cada joint y avisando si alguno pasa de 30°
  (mismo patrón que `cr5_go_home_demo.py`), y solo después saluda — dos
  confirmaciones por teclado, una por cada movimiento.

  Como la postura de saludo se conoce analíticamente, la IK del gesto
  ENTERO se resuelve antes de abrir siquiera la conexión: si algo no
  converge, se aborta sin haber tocado el robot. Incluye
  `_wait_until_robot_idle` tras el acercamiento y tras el saludo (hallazgo
  del 08/09 sobre `cp=50` y la cola de `MovJ`, ver más arriba).
  **Pendiente de ejecutar contra el robot físico** — requiere confirmación
  interactiva, no lanzado por el asistente.

## Primer contacto con hardware (`robot_node`, sin pasar por ROS2)

Hablan directo contra `RobotConnectorPort`/`Cr5RealRobotAdapter`, sin
ningún nodo ROS2 de por medio — aíslan "¿funciona el protocolo con ESTE
robot?" de "¿funciona el resto del stack?". Precursores de
`real_cr5_first_session_demo.py`. Ver [[Cr5RealRobotAdapter]] y
[[Conectar un Robot Nuevo]].

- **`cr5_first_contact_demo.py`** — primer contacto real, deliberadamente
  interactivo (pide confirmación por teclado en cada paso de riesgo
  creciente: leer estado → leer posición → mover un único joint un ángulo
  pequeño relativo a su posición actual → releer → des-energizar). Su
  docstring documenta por qué el primer chequeo de `RobotMode()` existe:
  sin él, una prueba anterior se encontró la conexión reseteada al llegar
  a `EnableRobot()` (ver el "CORREGIDO 04/09 (2)" en
  [[Cr5RealRobotAdapter]]).
- **`cr5_repeated_joint1_moves_demo.py`** — prueba de conexión
  PERSISTENTE: varios `MovJ` seguidos por el mismo socket TCP (uno por
  cada número escrito en terminal, 1-5°, sin confirmación extra por
  movimiento), para comprobar que la conexión aguanta una secuencia de
  comandos y no solo uno aislado — justo lo que hace falta para una
  trayectoria real de varios waypoints.
- **`cr5_disable_demo.py`** — herramienta de rescate: des-energiza el CR5
  con una conexión fresca cuando otro script lo dejó habilitado tras un
  fallo a mitad de prueba. **Corregido 08/09**: ahora intenta
  `GetErrorID()`→`ClearError()` primero — encontrado en vivo que
  `DisableRobot()` devuelve -2 ("robot en alarma") si no se limpia la
  alarma antes, algo que esta herramienta de rescate no hacía (ver
  [[Cr5RealRobotAdapter]]/[[SelfCollisionAwarePlanningAdapter]] sobre el
  incidente que lo reveló). Solo entonces prueba `DisableRobot()` directo
  (sin `RequestControl()` previo) bajo la hipótesis de que el modo TCP es
  estado del robot, no de la conexión — y solo si eso falla, cae a
  `RequestControl()`+`DisableRobot()` como último recurso.
- **`cr5_go_home_demo.py`** (08/09) — herramienta de preparación: mueve
  los 6 joints a la vez a la configuración inicial (home, 0° por defecto,
  la misma que usa `PoeKinematicsAdapter` como origen), en un único `MovJ`
  en vez de trocearlo. Muestra el recorrido de CADA joint y avisa si
  alguno supera un umbral (30° por defecto) antes de pedir confirmación —
  a diferencia de los otros demos de esta sección (deltas de 1-5°), aquí
  el recorrido puede ser grande según de dónde venga el robot. Se
  des-energiza al terminar, para que la sesión siguiente (p. ej. una
  prueba de varios objetivos consecutivos) pueda pedir
  `RequestControl()`/`EnableRobot()` desde cero. Verificado contra el
  arnés de servidor TCP de mentira de `test_cr5_real_adapter.py` (no
  contra el robot físico todavía) — pensado para correrse a mano,
  interactivo, nunca automatizado (necesita a alguien delante del botón
  de emergencia confirmando cada paso).

## PoE frente a GA (CGA/gafro) en CoppeliaSim (Bloque 1, 17/09)

- **`cr5_poe_vs_gafro_sim_demo.py`** (`commander`) — resuelve el mismo
  arco de `cr5_semicircle_sim_demo.py` con [[PoeKinematicsAdapter]] y
  [[GaKinematicsAdapter]] desde la home, anima ambas soluciones (rastro
  azul = PoE, verde = GA) y contrasta cada waypoint con la posición de
  `Link6_visual` que calcula el propio CoppeliaSim (verdad de terreno del
  URDF importado). Después, FK de los dos contra el simulador en 200
  posturas aleatorias. Escribe `docs/comparativa_poe_vs_gafro_coppeliasim.md`
  con la lectura generada a partir de los números. Usa
  `tip_position()`/`set_trail_color()` de [[CoppeliaSimRobotAdapter]] y
  `last_iteration_count` de los dos adaptadores (diagnóstico, no parte de
  [[KinematicsPort]]). Resultado y hallazgo (self-motion con `joint5=0`):
  en [[GaKinematicsAdapter]].
- **`cr5_poe_vs_gafro_simple_demo.py`** (`commander`) — la versión
  mínima, para ver las dos trayectorias a simple vista: (1) bajar 5 cm
  desde la home (muñeca singular: misma pose, articulaciones distintas,
  hasta ~2°); (2) desde una postura doblada (`joint5=40°`), 8 cm en -X y
  5 cm en -Z (solución aislada: misma configuración articular, diferencia
  0,01°). Error real de la punta en CoppeliaSim < 0,01 mm en los cuatro
  casos; GA ~1,5× más rápida por IK. Escena limpia al arrancar.

## Ver también

- [[Evitación de Colisiones]]
- [[CR5 vs Panda (Generalización)]]
- [[Commander y ControlSession]]
- [[Cr5RealRobotAdapter]]
- [[Estado del Roadmap]]
