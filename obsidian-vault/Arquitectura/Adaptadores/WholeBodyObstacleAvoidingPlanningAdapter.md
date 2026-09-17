---
tags: [arquitectura, adaptador]
---

# WholeBodyObstacleAvoidingPlanningAdapter

Segundo implementador de [[PlanningPort]], misma rama de experimentación
que [[ObstacleAvoidingPlanningAdapter]]: además del segmento del TIP,
comprueba que NINGÚN eslabón del robot (el segmento entre cada par de
articulaciones consecutivas, y entre `base_link` y la primera) invada
ningún obstáculo, en NINGÚN waypoint de la trayectoria. Código:
`src/controller_node/controller_node/adapters/whole_body_obstacle_avoiding_planning_adapter.py`.

Usa `KinematicsPort.link_poses` (hoy solo [[PoeKinematicsAdapter]] lo
ofrece) para saber dónde está CADA articulación en cada paso.

## Sigue sin ser CHOMP/RRT

Extensión iterativa de la estrategia de
[[ObstacleAvoidingPlanningAdapter]]: si el cuerpo completo invade algo en
algún waypoint, calcula un punto de paso para el TIP (el único lever de
control real — no se puede pedir "mueve el codo aquí" directamente) que
rodee al obstáculo peor invadido, con margen creciente
(`detour_growth_factor`) si no basta a la primera. Hasta que el cuerpo
quede libre o se agoten los intentos — sin garantía formal, devuelve el
mejor candidato encontrado.

## Hallazgo real: IK convergiendo a más de una vuelta

Al encadenar varias llamadas a `compute_trajectory` (cada intento de
desvío hace dos, cada una parte de donde terminó la anterior),
Newton-Raphson puede converger a un ángulo matemáticamente válido (misma
pose módulo 2π) pero MUY alejado de ±2π — visto en vivo: `joint4=-457°`,
`joint6=540°`. Como `JointConfiguration` no llevaba (en ese momento)
ningún límite físico, CoppeliaSim (que sí tiene los joints limitados)
recortaba esos ángulos EN SILENCIO — la trayectoria "convergía" pero el
robot acababa en una postura distinta a la calculada, sin ningún error.

`_within_a_full_turn` actúa de salvaguarda genérica: rechaza cualquier
candidato cuyo ángulo bruto pase de ±2π, tratándolo igual que una IK que
no converge. Precedente directo de los límites articulares que
`Cr5RealRobotAdapter._validate_joint_limits` implementó después contra el
robot real (ver [[Cr5RealRobotAdapter]]) — mismo tipo de error, dos capas
de defensa distintas.

## Ver también

- [[PlanningPort]]
- [[ObstacleAvoidingPlanningAdapter]]
- [[PoeKinematicsAdapter]]
- [[Cr5RealRobotAdapter]]
- [[Evitación de Colisiones]] — prueba real en vivo contra CoppeliaSim
- [[_segment_geometry (geometría compartida)]] — la geometría pura que usa por cada eslabón, vía `_body_segments`
