---
tags: [moc]
---

# robot-control-hexagonal — vault

Vault personal (`obsidian-vault/` dentro de este mismo repo desde el 07/09,
versionado junto al código) para entender, explicar y dejar constancia del
avance de `~/Desktop/robot-control-hexagonal`: arquitectura
hexagonal/DDD para controlar el brazo Dobot CR5 (CoppeliaSim y físico, ya
verificado ambos) sobre ROS2 Humble, con el objetivo de fondo de soportar un backend
"descripción en lenguaje natural → nodo robótico validado" (LLM + tools, no
generación de código libre).

Este vault es también mi primer proyecto real en Obsidian — si es la primera
vez que lo abres, empieza por [[Cómo usar este vault (Obsidian)]].

> [!info] Estado en una frase (07/09/2026)
> Cinemática PoE real, `Scene`/percepción real (fichero + pseudo-perceptor) y
> generalización a robot arbitrario (URDF) ya funcionan. El protocolo TCP/IP
> real contra el CR5 físico (Bloque 0, commit `e009321`) **ya se ha probado
> con éxito contra el robot físico de verdad** — primera IK real (PoE)
> ejecutada de punta a punta, no solo contra servidores de mentira.
> Detalle en [[Estado del Roadmap]].

## Arquitectura

- [[Arquitectura Hexagonal]] — capas, paquetes, dirección de dependencias
- [[Puertos y Adaptadores]] — los 5 puertos del dominio y quién los implementa
- [[Commander y ControlSession]] — la capa de aplicación y cómo se hablan los nodos
- [[Scene y Percepción]] — el agregado `Scene`, `PerceptionPort`, `RobotDescription`

## Avance del proyecto

- [[Estado del Roadmap]] — resumen por Bloque (0–12) del `ROADMAP.md` real
- [[Decisiones de Diseño Clave]] — decisiones no triviales, con fecha y motivo
- [[Diario]] — registro día a día de qué se hizo y en qué orden

## Pruebas y guías prácticas

- [[CR5 vs Panda (Generalización)]] — puesta a prueba con un robot de 7 GDL
- [[Evitación de Colisiones]] — los dos planificadores de obstáculos, en vivo
- [[Conectar un Robot Nuevo]] — guion genérico para dar de alta un robot distinto
- [[Anatomía de un Nodo]] — el pipeline YAML → nodo → callbacks, paso a paso
- [[Scripts de Demostración]] — catálogo de todos los scripts de demo/validación del repo, agrupados por tema

## Referencia de código (función por función)

Complemento de las notas de arquitectura de arriba: para cada paquete/nodo
principal, una nota que recorre sus funciones/métodos uno por uno (qué
hace, no cómo — el código en sí es la fuente de verdad). Útil para no
tener que releer un archivo entero para recordar qué hace una función
concreta.

- [[Value Objects y Dominio]] — `shared_kernel`: `Either`, errores de dominio, `JointConfiguration`, `Trajectory`
- [[Primitivas Geométricas]] — `geometry_kernel`: `Pose`, `Point`, `Plane`, `SphereObstacle`, métodos de `Scene`
- [[Infraestructura ROS2 (ros2_kit)]] — `messages.py`, `node_config.py`, `qos.py`, `runner.py` (+ el boceto descartado `ros1_kit`)
- [[urdf_kit y RobotDescription]] — cómo se deriva `RobotDescription` de un `.urdf` real
- [[RobotNode]] · [[ControllerNode]] · [[PerceptionNode]] — el `node.py` de cada nodo ROS2 real
- [[Commander (referencia de código)]] — `Commander` y `ControlSession`
- [[Herramientas de CoppeliaSim]] — `coppeliasim_launcher.py`/`coppeliasim_scene_builder.py`
- [[_cr5_protocol (protocolo TCP del CR5)]] — el módulo de protocolo detrás de [[Cr5RealRobotAdapter]]
- [[_segment_geometry (geometría compartida)]] — la geometría pura detrás de los dos planificadores de obstáculos

## Referencias técnicas (viven en el propio repo, no en este vault)

- `docs/nodos_ros2.md` — contrato de topics/QoS entre nodos
- `docs/configuracion_nodos.md` — parámetros YAML por nodo
- `docs/algebra_geometrica_conforme.md` — CGA, para cuando aterrice el Bloque 1
- `docs/pipeline_percepcion_planificacion.md` — pipeline percepción → planificación

## Meta

- [[Cómo usar este vault (Obsidian)]]
