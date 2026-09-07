---
tags: [arquitectura, adaptador]
---

# PoeKinematicsAdapter

Implementa [[KinematicsPort]] vía Product of Exponentials (Lynch & Park,
*Modern Robotics*) — la única cinemática REAL (no doble de test, no stub)
que resuelve IK de verdad hoy. Código:
`src/controller_node/controller_node/adapters/poe_adapter.py`. Derivación
completa función a función, con diagrama del mecanismo:
`poe_adapter.html` en el mismo directorio.

## De dónde salen los twists

`RobotDescription` (ver [[Scene y Percepción]]) en configuración home →
twist S_i=(w_i, v_i) por articulación: w_i = R_i·axis_i, v_i = -w_i×q_i
(o v_i = R_i·axis_i para prismáticas). Por defecto usa
`_DEFAULT_CR5_DESCRIPTION` — los mismos datos que antes vivían
hardcodeados como `_JOINT_NAMES`/`_JOINT_ORIGINS`, copiados del URDF real
del CR5.

`_validate_cr5_reference_if_applicable` es una red de seguridad TEMPORAL:
si un `RobotDescription` recibido dice ser un CR5 (mismos nombres de
joint), compara sus twists/pose home contra la referencia hardcodeada ya
validada a mano — confiar en la ruta genérica (`urdf_kit.parse_urdf_file`,
Bloque 9) para el único robot real que hay hoy, antes de fiarse de ella
para uno nuevo.

## IK: Newton-Raphson amortiguado, no pseudoinversa pura

`_inverse_kinematics` itera sobre el Jacobiano espacial (vía la Adjunta,
`IKinSpace` de *Modern Robotics* cap. 6), con paso amortiguado
(Levenberg-Marquardt) en vez de pseudoinversa sin más — el CR5 tiene
muñeca esférica y una singularidad real en joint5≈0 (ejes de joint4 y
joint6 paralelos ahí) donde la pseudoinversa sin amortiguar dispara el
paso. Si no converge en `max_iterations`, lanza `RuntimeError` explícito
en vez de devolver una trayectoria hacia un sitio equivocado.

## Dos métodos extra, fuera del puerto formal

- `forward_kinematics(configuration) -> Pose` — cinemática directa del
  tip, mismo marco (`base_link`) que exige `goal`. La usa
  [[ObstacleAvoidingPlanningAdapter]] para saber dónde está el robot
  AHORA en cartesiano.
- `link_poses(configuration) -> List[Pose]` — pose de CADA articulación,
  no solo el tip (`link_poses(...)[-1] == forward_kinematics(...)` por
  construcción). La usa [[WholeBodyObstacleAvoidingPlanningAdapter]] para
  comprobar colisiones de cuerpo completo, no solo del tip.

Ninguno de los dos es parte de [[KinematicsPort]] — el puerto solo exige
`compute_trajectory`; son capacidades extra que otros adaptadores
consumen si el `KinematicsPort` concreto que reciben las ofrece (duck
typing vía `Protocol` locales, ver `_KinematicsPortWithForward`/
`_KinematicsPortWithLinkPoses` en los planificadores).

## Ver también

- [[KinematicsPort]]
- [[ObstacleAvoidingPlanningAdapter]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[Scene y Percepción]] — `RobotDescription`
- [[CR5 vs Panda (Generalización)]] — prueba real con otro robot, 7 GDL
