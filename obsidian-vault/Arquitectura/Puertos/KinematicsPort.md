---
tags: [arquitectura, puerto]
---

# KinematicsPort

> `compute_trajectory(goal: Pose, current_configuration: JointConfiguration) -> Trajectory`

`shared_kernel/ports.py` — cinemática inversa PURA: de un objetivo
cartesiano a una trayectoria alcanzable, sin conocer la escena ni evitar
nada (eso es responsabilidad de [[PlanningPort]], que puede apoyarse en
un `KinematicsPort` para resolver cada tramo). `goal` es relativo a
`base_link` — no todos los adaptadores respetan ese marco de la misma
forma (ver el hallazgo de `two_sessions_demo.py` en [[Commander y ControlSession]]).

Dos métodos extra, NO parte formal del puerto (solo los ofrece
`PoeKinematicsAdapter` hoy), que consumen los adaptadores de
[[PlanningPort]] que necesitan saber DÓNDE está el robot en cartesiano:
`forward_kinematics(configuration) -> Pose` (solo el tip) y
`link_poses(configuration) -> List[Pose]` (cada articulación, no solo el
tip — `link_poses(...)[-1] == forward_kinematics(...)` por construcción).

## Adaptadores reales

- [[PoeKinematicsAdapter]] — Product of Exponentials, matemática propia.
- [[CoppeliaSimIkKinematicsAdapter]] — delega en `simIK` del simulador.

## Stubs (pendientes de verdad)

- **`GaKinematicsAdapter`** (`controller_node/adapters/ga_adapter.py`) —
  álgebra geométrica conforme (gafro). `compute_trajectory` lanza
  `NotImplementedError` — falta compilar `pygafro`/`gafro_ros` para el
  CR5 (Bloque 1). Ya guarda un `RobotDescription` opcional en el
  constructor, sin usar todavía.
- **`DhKinematicsAdapter`** (`controller_node/adapters/dh_adapter.py`) —
  Denavit-Hartenberg numérico clásico. `compute_trajectory` lanza
  `NotImplementedError` — falta extraer la tabla DH del CR5 e implementar
  el bucle Newton-Raphson/Levenberg-Marquardt ya deducido en las sesiones
  de teoría.

## Dobles de test (no resuelven IK de verdad)

- **`NaiveTestKinematicsAdapter`** (`controller_node/adapters/naive_test_adapter.py`)
  — ignora el objetivo por completo, aplica un barrido sinusoidal a la
  configuración actual. Solo para cablear el pipeline extremo a extremo —
  usado así el 07/09 para la primera validación real contra el CR5 físico
  (ver [[Estado del Roadmap]]), antes de probar PoE de verdad.
- **`StraightLineKinematicsAdapter`** (`controller_node/adapters/straight_line_adapter.py`)
  — SÍ mira el objetivo, pero con una "IK" arbitraria y determinista
  (`_approximate_inverse_kinematics`, no la cinemática real del CR5) para
  tener algo con lo que trazar una recta e interpolar.

## Ver también

- [[Puertos y Adaptadores]]
- [[PlanningPort]]
- [[Arquitectura Hexagonal]]
