# controller_node

Adaptador de entrada/salida ROS2 alrededor de `KinematicsPort`/`PlanningPort`:
de un `goal` cartesiano a una `Trajectory`, enviada como `joint_command`. Ver
`README.md` raíz y `docs/nodos_ros2.md` §1.3 para su responsabilidad exacta
(incluido el diseño objetivo de `PlannerSelectionPort`).

## Configuración de hoy (implementado)

Parámetros, topics y cómo sobreescribirlos: **`docs/configuracion_nodos.md`**
(tabla de `controller_node`). No se duplica aquí para no tener dos fuentes
de la verdad.

## Qué más suele hacer falta configurar en un nodo de este tipo (NO implementado hoy)

Varias de estas cosas YA existen como constante o argumento de constructor
dentro de un adaptador concreto — solo no están expuestas como parámetro
ROS2 todavía. Se listan con su valor actual exacto para que quede claro que
no es una idea abstracta, es código real sin YAML por delante.

- **Tolerancias de convergencia de la IK** (`poe`/`ga`) — hoy son
  argumentos del constructor de `PoeKinematicsAdapter`
  (`controller_node/adapters/poe_adapter.py`), con sus valores por
  defecto: `max_iterations=200`, `orientation_tolerance=1e-3`,
  `position_tolerance=1e-4`, `damping_factor=1e-2` (Levenberg-Marquardt).
  Nunca se leen de un parámetro — solo del propio código.
- **Margen de evitación de obstáculos** (`clearance`, `max_detour_attempts`,
  `detour_growth_factor`) — ya existen, con esos nombres exactos, como
  argumentos de `ObstacleAvoidingPlanningAdapter`/
  `WholeBodyObstacleAvoidingPlanningAdapter` (defaults `0.05`/`5`/`1.5`),
  pero `PlanningPort` todavía no está cableado dentro de `controller_node`
  (ver ROADMAP.md, Bloque 4/9) — así que hoy no hay ningún parámetro que
  los toque, solo el valor por defecto del adaptador si se instancia a mano.
- **Umbral de la salvaguarda de vuelta completa** — `_within_a_full_turn`
  (`whole_body_obstacle_avoiding_planning_adapter.py`) usa un ±2π fijo
  porque no hay límites reales por joint (ver README de `robot_node`); si
  esos límites llegan a existir, este umbral debería venir de ahí, no
  seguir siendo una constante genérica aquí.
- **Umbrales de `PlannerSelectionPort`** (cuándo pasar de una heurística
  rápida a un planificador de búsqueda más lento según lo cargada que esté
  la `Scene`) — puerto definido, sin ningún adaptador todavía (ROADMAP.md,
  Bloque 5). Serían parámetros de este nodo en cuanto exista.
- **Límites de velocidad/aceleración cartesianos o articulares** para la
  generación de trayectoria — hace falta con dinámica real (ROADMAP.md,
  Bloque 10); hoy `compute_trajectory` no tiene noción de tiempo entre
  waypoints más allá de `waypoint_period_seconds` (un intervalo fijo de
  publicación, no una velocidad calculada).
