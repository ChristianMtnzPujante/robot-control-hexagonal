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

- [[2026-10-01]] — se reorganiza `commander`: células descritas en YAML
  (`scenarios/`) y construidas sin scripts a medida, con dos modos de
  ejecución. Fase 1 hecha ([[Células y Escenarios]]), sin verificar en
  CoppeliaSim.
- [[2026-09-30]] — la pinza Robotiq 2F-85 funciona también en CoppeliaSim:
  URDF oficial montado en la brida del CR5 y [[CoppeliaSimGripperAdapter]].
  Decisión: un URDF por pieza, montadas por código, no uno combinado. Por
  la tarde, cuerpos en la escena y agarre cinemático.
- [[2026-09-29]] — la demo sencilla PoE vs GA explica el cálculo paso a
  paso. La única diferencia es la parte traslacional del log del motor
  (con giro, GA necesita más iteraciones). La diferencia de 2° en la home
  es por la singularidad, no por el álgebra. **Nuevo objetivo inicial**:
  artículo de revisión + propuesta de ontología del dominio, con escenas
  de ejemplo del repo ([[Ontología del Dominio (lenguaje CGA)]]). Por la
  tarde, **la pinza contesta por fin**: la vía era
  `ModbusCreate("127.0.0.1",60000,9,1)`, no `ModbusRTUCreate`.
- [[2026-09-24]] — pinza: el orden de configuración (alimentación antes
  de modo/formato) queda descartado, sigue en `-1`. El modo AI/485 del
  terminal no se puede leer. Adaptador USB-Ethernet en bucle de
  desconexión.
- [[2026-09-23]] — diseño, sin código de producción: GA hoy solo cubre el
  `KinematicsPort`. Decisión abierta: tareas CGA por primitivas en un
  puerto aparte (residuo + Jacobiano), el meet para fusionar restricciones
  sobre un mismo punto, y planificador separado del ejecutor. Ejemplos
  numéricos sobre el CR5 en `docs/cga_*.py`.
- [[2026-09-21]] — la pinza, acotada a tres candidatos: el sensor ATI queda
  fuera del bus (medida con la pinza desenchufada), se corrige la lectura de
  la polarización del 18/09 (los canales analógicos tenían 60 mV de desfase
  entre sí) y se prepara `cr5_485_scope_test.py` para mirar la señal con
  osciloscopio.
- [[2026-09-18]] — arquitectura del controlador del CR5 (puertos de red vs
  puertos físicos, el controlador como puente Modbus), `GripperPort` +
  `Robotiq2FGripperAdapter` con prueba por topic, y diagnóstico cerrado de
  por qué la pinza no contesta: LED rojo fijo = `gFLT 0x09`, está viva pero
  no le llegan los datos — es el cable.
- [[2026-09-17]] — `GaKinematicsAdapter` real sobre `pygafro` (F1.2 de la
  tesis) tras cerrar la prueba de viabilidad F1.1 por la mañana: no hay
  nada que compilar, es una rueda de PyPI. Comparativa PoE vs GA en
  CoppeliaSim: misma pose, GA más rápida, y las dos IK reparten distinto la
  redundancia de muñeca. Y al final del día, sondeo de la pinza por el
  conector de 8 pines del extremo: no contesta ni por 485 ni por las E/S
  digitales — ver [[E-S del Extremo del CR5 (pinza)]].
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
