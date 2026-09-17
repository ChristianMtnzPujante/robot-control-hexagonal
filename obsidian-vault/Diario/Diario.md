---
tags: [diario, moc]
---

# Diario

Registro día a día de qué se hizo y por qué, con enlaces a la nota de
arquitectura/decisión donde vive el detalle técnico — el diario cuenta el
ORDEN de los hechos, las notas de [[Arquitectura Hexagonal|Arquitectura]] y
[[Decisiones de Diseño Clave|Decisiones]] cuentan el estado actual. Una
entrada nueva por cada día de trabajo real, no por cada mensaje.

Más reciente primero:

- [[2026-09-14]] — gesto de saludo con el CR5 (sim verificado, físico
  pendiente): arco de lado a lado con la herramienta inclinada hacia
  arriba, tras una corrección del usuario sobre una primera versión recta
  y horizontal. Dos hallazgos: la home está en el borde del alcance
  (0.933m contra 0.9m de catálogo), y conviene anclar la rama de la IK en
  espacio de articulaciones en vez de confiar en que converja a la misma.
- [[2026-09-08]] — capa de referencia de código función por función para
  todo el repo; `forward_kinematics`/`link_poses` parte formal de
  `KinematicsPort`; pseudo-perceptor cableado de verdad en
  `perception_node`; scripts de semicírculo/círculo completo contra el
  CR5 (sim y real) con tres hallazgos reales por el camino — autocolisión
  real (`GetErrorID()`=[76]) y nuevo `SelfCollisionAwarePlanningAdapter`/
  `SelfCollisionAvoidingPlanningAdapter` para detectarla y esquivarla;
  desvío de muñeca al cerrar el círculo; y cierre/lectura prematuros
  antes de que el robot terminara de moverse (mismo patrón que el
  cierre prematuro del 07/09).
- [[2026-09-07]] — pruebas físicas contra el CR5 (vibración, proceso
  zombie, cierre prematuro de sesión, límites articulares) + auditoría y
  reorganización completa del vault, traído dentro del repo.
- [[2026-09-04]] — primera conexión real al CR5 físico: `RequestControl()`,
  primer movimiento confirmado, socket de comandos muriendo por
  inactividad.
- [[2026-09-03]] — configuración declarativa de nodos ROS2 (YAML →
  `node_config.py`), `joint_names` derivado de un URDF.
- [[2026-09-01]] — dos planificadores de evitación de obstáculos
  (tip-only y cuerpo completo), fusionados a `main`.
- [[2026-08-31]] — generalización a URDF real, esqueleto de
  percepción/planificación, canal de cambio de estrategia en caliente.

## Cómo se mantiene esto

Al final de una sesión de trabajo real (no de cada mensaje suelto), una
entrada nueva aquí con lo que se hizo, en el orden en que pasó, enlazando a
la nota de arquitectura/decisión que ya tenga el detalle en vez de
repetirlo — ver también la instrucción equivalente para ROADMAP.md/Vikunja
en `CLAUDE.md`, en la raíz del repo.

## Ver también

- [[Home]]
- [[Estado del Roadmap]]
- [[Decisiones de Diseño Clave]]
