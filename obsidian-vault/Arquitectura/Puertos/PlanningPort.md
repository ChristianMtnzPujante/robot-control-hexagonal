---
tags: [arquitectura, puerto]
---

# PlanningPort

> `compute_trajectory(goal: Pose, current_configuration: JointConfiguration, scene: Scene) -> Trajectory`

`shared_kernel/ports.py` — como [[KinematicsPort]], pero consciente de la
`Scene` (obstáculos, planos) y debe evitarlos. Puede apoyarse en un
`KinematicsPort` para resolver IK punto a punto (los dos adaptadores
reales de hoy lo hacen así), o resolver todo en espacio de articulaciones
— la interfaz no lo impone.

**Ninguno de los adaptadores reales de hoy es CHOMP/RRT** — son
heurísticas geométricas deterministas ("si la recta choca, calcula un
punto de paso que rodee el obstáculo"), no técnicas de búsqueda/IA. CHOMP
y RRT (pendientes, Bloque 4) son, con diferencia, el primer hito real
hacia el objetivo de formación en IA de la beca.

## Adaptadores reales

- [[ObstacleAvoidingPlanningAdapter]] — evita un obstáculo mirando solo
  la trayectoria del TIP.
- [[WholeBodyObstacleAvoidingPlanningAdapter]] — evita mirando CADA
  eslabón del robot, no solo el tip.

## Doble de test

- **`NaivePlanningAdapter`** (`controller_node/adapters/naive_planning_adapter.py`)
  — envuelve un `KinematicsPort` e ignora la `Scene` por completo. NO
  evita nada — deja el puerto cableado de punta a punta antes de invertir
  en un planificador real, mismo papel que `NaiveTestKinematicsAdapter`
  tuvo para `KinematicsPort`.

## Pendiente (Bloque 4)

CHOMP (gradiente) y RRT (muestreo), como nuevas `strategy` de
`controller_node` — mismo patrón de registro que ya usa `_build_adapter`.

## Ver también

- [[Puertos y Adaptadores]]
- [[KinematicsPort]]
- [[Estado del Roadmap]]
