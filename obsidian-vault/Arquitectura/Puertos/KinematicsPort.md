---
tags: [arquitectura, puerto]
---

# KinematicsPort

> `compute_trajectory(goal: Pose, current_configuration: JointConfiguration) -> Trajectory`
> `forward_kinematics(configuration: JointConfiguration) -> Pose`
> `link_poses(configuration: JointConfiguration) -> List[Pose]`

`shared_kernel/ports.py` — cinemática de un robot de cadena serie: de un
objetivo cartesiano a una trayectoria alcanzable (inversa, `compute_trajectory`),
y de una `JointConfiguration` a dónde está el robot en cartesiano (directa,
los otros dos), sin conocer la escena ni evitar nada (eso es responsabilidad
de [[PlanningPort]], que puede apoyarse en un `KinematicsPort` para resolver
cada tramo). `goal`/las poses devueltas son relativas a `base_link` — no
todos los adaptadores respetan ese marco de la misma forma (ver el
hallazgo de `two_sessions_demo.py` en [[Commander y ControlSession]]).

## Cinemática directa: parte formal del contrato desde el 08/09

`forward_kinematics` (solo el tip) y `link_poses` (cada articulación, no
solo el tip) son, conceptualmente, algo que CUALQUIER `KinematicsPort` de
una cadena serie debería poder dar, resuelva la IK como la resuelva — la
cinemática directa es un cálculo mucho más simple que la inversa (una
composición de transformaciones, sin iterar). Hasta el 08/09 NO eran
parte del `Protocol` (solo `compute_trajectory` lo era) y solo las
implementaba `PoeKinematicsAdapter` — cada consumidor de [[PlanningPort]]
que las necesitaba declaraba su propio `Protocol` local más estrecho
(`_KinematicsPortWithForward`/`_KinematicsPortWithLinkPoses`, uno por
adaptador) para exigirlas por duck typing. Formalizado en el puerto mismo
— ver [[Decisiones de Diseño Clave]] para el motivo y la fecha.

No se garantiza `link_poses(...)[-1] == forward_kinematics(...)` a nivel
de puerto — coincide en PoE (`RobotDescription` no tiene ningún eslabón
estático tras la última articulación), pero un adaptador cuyo `tip` tenga
un offset propio (una malla/dummy más allá del último joint) puede
legítimamente devolver poses distintas.

| Adaptador                                                    | `forward_kinematics`/`link_poses`                                                                                                                                                                                                          |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [[PoeKinematicsAdapter]]                                     | Real — matemática propia (screw axes), `link_poses(...)[-1] == forward_kinematics(...)`.                                                                                                                                                   |
| [[CoppeliaSimIkKinematicsAdapter]]                           | Real desde el 08/09 — sobre el mismo entorno IK aislado que `compute_trajectory` (`simIK.setJointPosition` + `simIK.getObjectPose`, sin tocar la escena real). **Sin verificar en vivo contra CoppeliaSim en esta sesión** (requiere GUI). |
| `GaKinematicsAdapter`/`DhKinematicsAdapter`                  | `NotImplementedError`, igual que `compute_trajectory` — siguen siendo stubs.                                                                                                                                                               |
| `NaiveTestKinematicsAdapter`/`StraightLineKinematicsAdapter` | `NotImplementedError` — dobles de test sin modelo geométrico real; no emparejar con un [[PlanningPort]] que necesite esto.                                                                                                                 |

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
- [[Value Objects y Dominio]] — referencia función por función de `Trajectory`/`JointConfiguration`, lo que produce y consume este puerto
