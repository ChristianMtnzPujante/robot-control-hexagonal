---
tags: [arquitectura, adaptador]
---

# Cr5RealRobotAdapter

Implementa [[RobotConnectorPort]] contra el CR5 físico real, hablando su
protocolo TCP/IP directo — no un puente ROS1 al driver oficial (ver
[[Decisiones de Diseño Clave]], "CR5 físico: TCP/IP directo, no puente
ROS1"). Código: `src/robot_node/robot_node/adapters/cr5_real_adapter.py`
+ `_cr5_protocol.py` (el protocolo en sí, sin nada de dominio).

**Verificado contra el robot físico real el 07/09** — primera IK real
(PoE) ejecutada con éxito de punta a punta, no solo contra servidores TCP
de mentira.

## Dos sockets, dos protocolos distintos

- **Comandos** (`Cr5CommandSocket`, puerto 29999 "Dashboard") — ASCII,
  petición/respuesta: `EnableRobot()` → `0,{},EnableRobot();`. TODOS los
  comandos van por aquí, incluido el movimiento (`MovJ`) — no hay un
  puerto de movimiento separado (ver hallazgo del driver de 2021 más
  abajo).
- **Tiempo real** (`Cr5RealtimeSocket`, puerto 30004) — stream binario
  continuo de tramas de 1440 bytes (`q_actual`, la posición articular
  real, en el byte 432). `_extract_last_frame` busca la trama MÁS
  RECIENTE del buffer, no la primera que llegue — necesario porque el
  robot transmite continuamente y el buffer puede acumular varias tramas
  entre lecturas.

## El protocolo NO era el del driver de 2021

Hallazgo real (ver [[Decisiones de Diseño Clave]] para el detalle
completo): la primera versión se basó en un driver de referencia de 2021
(`dobot_bringup`) que asumía un puerto de movimiento aparte (30003) con
`JointMovJ(j1,...,j6)`. El manual oficial del fabricante (2025) dice otra
cosa: solo existen 29999/30004/30005/30006, y el movimiento va como
`MovJ(joint={j1,...,j6},cp=<valor>)` por el MISMO puerto 29999.

`RequestControl()` — descubierto en la primera prueba real, no en la
lectura del manual: hay que pedirlo ANTES de cualquier otro comando TCP,
incluido `EnableRobot()`. Solo se admite si el robot está sin energizar o
des-energizado. Confirmado en vivo: es un estado del ROBOT, no de la
conexión concreta que lo pidió — una conexión nueva puede mandar
`DisableRobot()` sin volver a pedir `RequestControl()`.

## Reconexión y reintento (`Cr5CommandSocket`)

Conecta sola si hace falta, y si un envío falla por un problema de
conexión, reconecta y reintenta UNA vez antes de rendirse — seguro porque
todo el protocolo es idempotente (`MovJ` manda posiciones absolutas, no
deltas). Motivo: la conexión puede morir por inactividad (varios segundos
sin tráfico) o por latencia de la primera respuesta tras un rato sin
hablarle al robot — ambos vistos en vivo repetidamente.

## `cp` — suavizado entre waypoints

`_DEFAULT_MOVJ_CP = 50`. Sin especificarlo, el robot usa 0 (sin
suavizado) — para completamente en cada waypoint de una trayectoria
multi-punto, lo que se sintió como vibración y lentitud reales en una
prueba contra el robot físico (07/09). 50 es un valor intermedio
deliberado: con `cp>0` el robot no pasa exactamente por los puntos
intermedios, así que el máximo (100) se descartó a propósito.

**Configurable por YAML/ROS2** (`cr5_movj_cp` en `robot_node.yaml`,
pasante también en `ControlSession`/`Commander.create_session`) — a
diferencia del límite de FÁBRICA de más abajo, que nunca se puede
ampliar. Es una preferencia de ajuste de movimiento, no un dato de
seguridad, así que sí tiene sentido que pueda pisarse por sesión (ver
[[Decisiones de Diseño Clave]], "config vs. constante").

## Límites articulares (`_validate_joint_limits`)

Rechaza con `Cr5ProtocolError` ANTES de energizar el robot si algún
ángulo excede el límite EFECTIVO de su joint. `_FACTORY_JOINT_LIMITS_DEGREES`
(±360° en J1/J2/J4/J5/J6, ±160° en J3 — el codo, el único realmente
restrictivo) es el TECHO real, verificado contra tres fuentes oficiales
independientes (manual de usuario, manual de hardware, página de
producto), no solo la URDF local.

**Configurable, pero solo para ESTRECHAR** (`joint_limits_degrees` en el
constructor, `cr5_joint_limits_degrees` en `robot_node.yaml` — pedir un
valor mayor que la fábrica no tiene efecto, se toma el mínimo joint a
joint). Pensado para dos casos: un robot NUEVO define su propio límite de
fábrica en su propio adaptador (no reutiliza esta constante), y una
sesión concreta del MISMO CR5 puede pedir un margen más cauto (p. ej.
cerca de una persona) sin poder, por typo o descuido, ampliarlo por
encima del límite mecánico real — ver
[[Decisiones de Diseño Clave]], "config vs. constante".

Defensa en profundidad, no solo teórica: el mismo proyecto ya tuvo un bug
real de IK convergiendo a más de una vuelta (ver
[[WholeBodyObstacleAvoidingPlanningAdapter]]) que esto habría atrapado
igual de bien.

**Sin resolver todavía**: validación de velocidad/salto entre waypoints
consecutivos (más compleja — necesita conocer el `dt` real entre ellos).

## `close()` y el ciclo de vida completo

`close()` des-energiza el robot si seguía habilitado (mejor esfuerzo:
cierra los sockets pase lo que pase, pero relanza el error si `disable()`
falló). `RobotNode.destroy_node()` lo llama — cubre tanto el cierre
normal de una `ControlSession` como un crash (ver [[Commander y ControlSession]]).

## Ver también

- [[RobotConnectorPort]]
- [[CoppeliaSimRobotAdapter]]
- [[Decisiones de Diseño Clave]]
- [[Estado del Roadmap]]
