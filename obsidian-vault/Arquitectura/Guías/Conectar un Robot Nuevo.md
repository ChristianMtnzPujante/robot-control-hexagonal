---
tags: [arquitectura, guia]
---

# Conectar un Robot Nuevo

Generalización del guion de conexión real construido para el CR5
([[Cr5RealRobotAdapter]], Bloque 0) a la pregunta general: ¿qué hace falta
para enchufar cualquier otro robot físico distinto a esta misma
arquitectura? La separación por puertos ya resuelve buena parte del
problema antes de escribir una sola línea de adaptador nuevo.

## Qué reduce el hexágono, qué no

La arquitectura no reduce el trabajo de integrar un robot nuevo a cero —
pero sí lo reduce a un único punto de contacto bien definido, en vez de un
cambio disperso por todo el sistema.

**Ya genérico, cero trabajo nuevo** (gracias a los puertos + `urdf_kit`):

- [[RobotConnectorPort]] — única interfaz que un robot nuevo tiene que
  cumplir: `set_joints` / `get_current_configuration` / `close`.
- `urdf_kit/parser.py` — deriva twists, nombres y DOF de cualquier `.urdf`
  válido (joints revolute/continuous/prismatic/fixed). Cambiar de robot es
  cambiar de fichero, no de código.
- `ControlSession` / namespacing / QoS — agnóstico al robot: el mismo
  grafo de procesos ROS2 sirve para cualquiera.
- Registro de adaptadores en `robot_node` (`_TARGETS`, ver
  [[Anatomía de un Nodo]]) — añadir un robot es registrar una entrada
  nueva en el dict, no una rama `elif` nueva.

**Específico por robot, irreducible** (pero con forma genérica):

- Implementar el *driven adapter* contra el protocolo/SDK real del
  fabricante — cada uno habla su propio idioma.
- Checklist físico y de red de ESE hardware concreto (IP, puertos,
  procedimiento de emergencia).
- Validación end-to-end contra ESE robot — la física real no se deduce de
  la arquitectura.

## El guion genérico, en 5 pasos

Extraído de haberlo recorrido una vez, de forma concreta, para el CR5 —
ver la tabla de instanciación más abajo.

1. **Decidir la estrategia de interfaz.** ¿El fabricante da SDK/driver
   nativo ROS2, un driver ROS1 que hay que puentear, o solo un protocolo
   de bajo nivel (TCP/serie) que hay que reimplementar a mano? La
   respuesta determina todo lo siguiente — si ya es compatible con ROS2
   el trabajo se simplifica mucho, y este paso es el que más tiempo puede
   ahorrar o costar.
2. **Implementar el adaptador.** Una clase nueva que cumpla
   [[RobotConnectorPort]] contra la vía elegida. Es la única pieza
   realmente nueva — todo lo demás del sistema no necesita saber que
   existe un robot distinto.
3. **Cablear la fábrica de `robot_node`.** Pasarle al adaptador ganador
   sus parámetros de conexión reales (host/puertos, o los topics de un
   puente) — ver [[Anatomía de un Nodo]] para el mecanismo completo
   YAML → parámetro → adaptador. Un `(host, port)` genérico casi nunca
   basta.
4. **Checklist físico y de red.** Antes del primer movimiento real:
   IP/puertos alcanzables desde la máquina que corre `robot_node`,
   procedimiento de parada de emergencia probado, zona despejada,
   velocidad reducida.
5. **Validación end-to-end.** Mover el robot real a través de todo el
   stack (Commander → `ControlSession` → `controller_node` → `robot_node`
   → adaptador) y contrastar contra una fuente de verdad independiente —
   el propio panel/controlador del fabricante.

## Instanciación concreta: el CR5

Cómo se materializó cada paso genérico para este robot en particular
(`ROADMAP.md`, Bloque 0):

| Paso | Instanciación para el CR5 |
|---|---|
| 1 | El driver oficial (`dobot_bringup`) es ROS1/catkin puro → se decidió reimplementar el protocolo TCP/IP directo en vez de puentear ROS1 (ver [[Decisiones de Diseño Clave]], "CR5 físico: TCP/IP directo, no puente ROS1"). |
| 2 | [[Cr5RealRobotAdapter]] — dos sockets TCP (29999 comandos, 30004 tiempo real); NO existe el puerto 30003 de movimiento que asumía el driver de referencia de 2021 (corrección documentada en [[Decisiones de Diseño Clave]]). |
| 3 | `_build_adapter`/`_TARGETS` en `robot_node/node.py`, entrada del CR5 real: pasa `cr5_host`, `cr5_movj_cp`, `cr5_joint_limits_degrees` — no un único `(host, port)` genérico. |
| 4 | IP real confirmada contra el teach pendant; `RequestControl()` y el e-stop probados antes del primer `MovJ`. |
| 5 | Primer `MovJ` conservador de punta a punta, validado contra el propio dashboard del CR5 — verificado contra el robot físico el 07/09 (ver [[Cr5RealRobotAdapter]]). |

## Ver también

- [[RobotConnectorPort]]
- [[Cr5RealRobotAdapter]]
- [[Anatomía de un Nodo]]
- [[Decisiones de Diseño Clave]]
- [[urdf_kit y RobotDescription]] — referencia función por función de cómo `urdf_kit` deriva twists/nombres/DOF de cualquier `.urdf`
- [[RobotNode]] — dónde se cablea de verdad `_TARGETS` para un robot nuevo
