---
tags: [arquitectura, adaptador]
---

# ObstacleAvoidingPlanningAdapter

Primer implementador real de [[PlanningPort]]: evita un único
`SphereObstacle` desviando la línea recta cartesiana con UN punto
intermedio, delegando en el [[KinematicsPort]] recibido para resolver
cada tramo. Código:
`src/controller_node/controller_node/adapters/obstacle_avoiding_planning_adapter.py`.
Rama `experimento/planificador-evita-obstaculo`.

**No es CHOMP ni RRT** — no hay gradiente, no hay optimización, no
maneja varios obstáculos a la vez (si la `Scene` trae varios, solo se
esquiva el que más invade el segmento recto). Precisión sobre qué tipo de
planificador ES, dos ejes distintos:

- **Cómo decide**: no busca — corrección local, en espíritu un campo de
  potenciales (Khatib 1986) de UN SOLO PASO, calculado analíticamente una
  vez.
- **Cuándo decide**: NO reactivo en tiempo real — la `Scene` se da
  entera y fija de antemano, se calcula la trayectoria COMPLETA antes de
  mandar el primer waypoint. La parte de verdad reactiva
  ("Replanificación local cuando cambia el campo de obstáculos", Bloque
  4) sigue sin implementarse.

## Requiere `forward_kinematics`

El `KinematicsPort` recibido debe exponer también `forward_kinematics`
(hoy solo [[PoeKinematicsAdapter]] lo hace) — sin eso no hay forma de
saber dónde está el efector en cartesiano para comprobar si el segmento
pasa cerca del obstáculo.

## Algoritmo, en 4 pasos (`compute_trajectory`)

1. `forward_kinematics(current_configuration)` — dónde está el tip AHORA.
2. `worst_intersection` (`_segment_geometry.py`) — ¿la recta start→goal
   invade algún obstáculo? Si no, vía libre: IK directa, sin más.
3. Si hay colisión: `detour_point` calcula UN punto de paso que rodee al
   obstáculo que peor invade, conservando la orientación del goal (no se
   interpola orientación).
4. Dos tramos, cada uno resuelto por el `KinematicsPort` — nunca por este
   planificador — concatenados sin duplicar el waypoint de unión.

Solo mira el segmento del TIP — un obstáculo puede seguir chocando con el
codo/antebrazo aunque el tip lo esquive. Para eso, ver
[[WholeBodyObstacleAvoidingPlanningAdapter]].

## Ver también

- [[PlanningPort]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[PoeKinematicsAdapter]]
- [[Evitación de Colisiones]] — prueba real en vivo contra CoppeliaSim
