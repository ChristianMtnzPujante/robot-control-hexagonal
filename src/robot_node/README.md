# robot_node

Adaptador de entrada/salida ROS2 alrededor de `RobotControllerPort`: ejecuta
`joint_command`, reporta `joint_states`. No decide nada — ver `README.md`
raíz y `docs/nodos_ros2.md` §1.2 para su responsabilidad exacta.

## Configuración de hoy (implementado)

Parámetros, topics y cómo sobreescribirlos: **`docs/configuracion_nodos.md`**
(tabla de `robot_node`). No se duplica aquí para no tener dos fuentes de la
verdad — ese documento es el que se mantiene al día.

## Qué más suele hacer falta configurar en un nodo de este tipo (NO implementado hoy)

Ideas con sentido para cuando este nodo tenga que controlar el robot de
verdad, no solo mover joints "porque el mensaje lo dice". Ninguna existe
todavía — se listan aquí para no perderlas y para que, si se implementan,
se sepa que van en el YAML de este nodo (`config/robot_node.yaml`), no
sueltas en código.

- **Límites por articulación** (mín/máx ángulo). Hoy `JointDescription`
  (`shared_kernel/robot_description.py`) no tiene ningún campo `limits` —
  el hueco ya es visible indirectamente en
  `WholeBodyObstacleAvoidingPlanningAdapter._within_a_full_turn`, que tuvo
  que añadir una salvaguarda genérica de ±2π precisamente porque no hay un
  límite real por joint contra el que comprobar (ver ROADMAP.md, Bloque 9).
  Con límites reales, `robot_node` podría rechazar/recortar un
  `joint_command` fuera de rango ANTES de mandarlo al adaptador — una
  frontera de seguridad de verdad, no solo un dato documentado.
- **Límites de velocidad/aceleración por articulación** — hace falta en
  cuanto haya un controlador de bajo nivel real (ROADMAP.md, Bloque 11) o
  dinámica (Bloque 10); hoy `set_joints` no tiene noción de "qué tan rápido".
- **Postura de reposo/home** — a dónde ir al arrancar o ante un stop/error.
  Hoy es implícito (lo que ya traiga la escena de CoppeliaSim cargada), no
  una decisión explícita del nodo.
- **Watchdog de `joint_command`**: qué hacer si no llega ningún mensaje
  durante N segundos — ¿mantener la última posición, ir a una postura
  segura? Hoy no hay ningún timeout, el robot simplemente se queda quieto
  sin que nadie lo decida a propósito.
- **Conexión del robot real** (`host`/`port` de `Cr5RealRobotAdapter`) —
  hoy hardcodeados en `_build_adapter` (`192.168.1.100:29999`), con el
  propio código ya señalando que debería ser configuración (ver
  ROADMAP.md, Bloque 9).
- **Offset de la herramienta/efector final** (y estado de una pinza, si la
  hay) — no existe ningún modelado de efector todavía.
