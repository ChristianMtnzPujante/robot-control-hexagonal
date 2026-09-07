---
tags: [moc]
---

# robot-control-hexagonal — vault

Vault personal (fuera del repo, no versionado) para entender, explicar y dejar
constancia del avance de `~/Desktop/robot-control-hexagonal`: arquitectura
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

## Referencias técnicas (viven en el propio repo, no en este vault)

- `docs/nodos_ros2.md` — contrato de topics/QoS entre nodos
- `docs/configuracion_nodos.md` — parámetros YAML por nodo
- `docs/algebra_geometrica_conforme.md` — CGA, para cuando aterrice el Bloque 1
- `docs/pipeline_percepcion_planificacion.md` — pipeline percepción → planificación

## Meta

- [[Cómo usar este vault (Obsidian)]]
