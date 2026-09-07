---
tags: [arquitectura]
---

# Puertos y Adaptadores

Los cinco puertos viven en `shared_kernel/ports.py`, todos como
`typing.Protocol`. Ver [[Arquitectura Hexagonal]] para el porqué de esa
elección. Esta nota es solo el mapa — cada puerto tiene su propia página
con la firma completa y el desarrollo de cada adaptador (varios con
referencias a código y a hallazgos reales).

## [[RobotConnectorPort]] — el "nodo robot"

`set_joints`, `get_current_configuration`, `close()`. Nunca calcula nada,
solo obedece y reporta.

| Adaptador | Estado |
|---|---|
| [[CoppeliaSimRobotAdapter]] | Real, funcional |
| [[Cr5RealRobotAdapter]] | Real, **verificado contra el robot físico (07/09)** |

## [[KinematicsPort]] — el "nodo controlador"

`compute_trajectory(goal, current_configuration) -> Trajectory`. IK pura,
sin conocer la escena ni evitar nada.

| Adaptador | Estado |
|---|---|
| [[PoeKinematicsAdapter]] | Real — Product of Exponentials |
| [[CoppeliaSimIkKinematicsAdapter]] | Real — delega en `simIK` del propio simulador |
| `GaKinematicsAdapter` | Stub (`NotImplementedError`) — pendiente de CGA/`gafro`, Bloque 1 |
| `DhKinematicsAdapter` | Stub (`NotImplementedError`) — pendiente de tabla DH, Bloque 9 |
| `NaiveTestKinematicsAdapter` / `StraightLineKinematicsAdapter` | Dobles de test, solo cablean el flujo |

Detalle de los stubs/dobles de test: ver [[KinematicsPort]].

## [[PlanningPort]] — como `KinematicsPort` pero consciente de la escena

`compute_trajectory(goal, current_configuration, scene: Scene) -> Trajectory`.

| Adaptador | Estado |
|---|---|
| [[ObstacleAvoidingPlanningAdapter]] | Real — heurística geométrica (evita solo la trayectoria del tip) |
| [[WholeBodyObstacleAvoidingPlanningAdapter]] | Real — evita con el cuerpo completo, no solo el tip |
| `NaivePlanningAdapter` | Doble de test — ignora la `Scene` por completo |
| CHOMP / RRT | Pendientes — Bloque 4. Los tres adaptadores de arriba son heurísticas/dobles deterministas, **no** técnicas de IA/búsqueda — CHOMP/RRT son el primer hito real hacia el objetivo de formación en IA. |

## [[PlannerSelectionPort]] — elige estrategia de planificación

`select(scene: Scene) -> str`. `FixedPlannerSelectionAdapter` — único
adaptador hoy, devuelve siempre la misma estrategia sin mirar la
`Scene` (la selección real, HyperPlan, es Bloque 5).

## [[PerceptionPort]] — lo que el sistema sabe de la escena

`get_scene() -> Scene`. Devuelve una `Scene` completa de una vez (no
streaming). Ver [[Scene y Percepción]] para el agregado en sí.

| Adaptador | Estado |
|---|---|
| `StaticPerceptionAdapter` | Real — fijo desde construcción |
| [[FilePerceptionAdapter]] | Real — relee un fichero de texto entero en cada `get_scene()` |
| [[PseudoPerceptionAdapter]] | Real — permite "inyectar" eventos con el tiempo, sin cámara real |
| Adaptador CoppeliaSim (ground truth simulado) | Pendiente — sin precedente en el repo de leer el radio de una esfera vía la API ZMQ |
| Cámara real | Pendiente, bloqueado detrás de grounding (Bloque 3) |

## Ver también

- [[Arquitectura Hexagonal]]
- [[Commander y ControlSession]]
- [[Estado del Roadmap]]
