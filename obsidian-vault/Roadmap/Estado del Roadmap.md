---
tags: [roadmap]
---

# Estado del Roadmap

Resumen del `ROADMAP.md` real del repo (objetivo de fondo: backend
"descripción en lenguaje natural → nodo robótico validado", ver [[Home]]).
No sustituye al ROADMAP — es la foto rápida para explicar en qué punto está
el proyecto sin leer 560 líneas. El detalle línea a línea vive en el propio
`ROADMAP.md`.

Orden de dependencias: Bloques 0–2 en paralelo desde el principio (0 es
código, 1–2 son lectura). 3 y 4 en paralelo una vez cerrado 0. 5 depende de
4. 6 depende de 3+4/5. 7 depende de 6. 8 al final. 9 no bloquea a ningún
otro pero conviene tenerlo en cuenta cuanto antes. 10–12 no bloquean nada
anterior.

> [!success] Verificado contra el robot físico (07/09/2026)
> El código de `e009321` ("Bloque 0: protocolo TCP/IP real contra el CR5
> físico + fix de proceso zombie") se probó ese mismo día contra el CR5
> físico real, no solo contra servidores de mentira: primera IK real con
> PoE (subir/bajar el TCP unos centímetros, con y sin girar joint6)
> ejecutada con éxito a través de TODO el stack (Commander → ControlSession
> → controller_node → robot_node → Cr5RealRobotAdapter → CR5), con la
> posición final coincidiendo EXACTAMENTE con la predicción de PoE.
>
> Por el camino, varios hallazgos reales más (todos corregidos y ya
> commiteados en `e009321` también, pese a que el mensaje del commit no
> los detalla uno a uno): límites articulares en `set_joints` (±360° la
> mayoría de joints, ±160° el codo — verificados contra tres fuentes
> oficiales, no solo la URDF), `MovJ` sin el parámetro `cp` (suavizado)
> causaba vibración/lentitud real, el cierre de sesión no esperaba a que
> el robot real terminara de procesar la cola antes de matar el proceso, y
> un error "-7 script pausado" recurrente que solo se resolvió con un
> power-cycle físico del controlador (probable estado interno acumulado
> tras un día especialmente intenso de pruebas: corte de red, incidente de
> e-stop, procesos zombies). Detalle completo en Vikunja #110/#114/#116/#117/#118
> y en `ROADMAP.md` (Bloque 0).

## Por bloque

| Bloque                                             | Estado              | Resumen                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| -------------------------------------------------- | ------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0 — CR5: instanciación y caso de uso real          | #bloque/en-progreso | Cinemática real (PoE) hecha. Protocolo TCP/IP real (`e009321`, incluye límites articulares/`cp`/manejo de errores/fix de timing) **verificado en vivo contra el CR5 físico el 07/09** — primera IK real ejecutada con éxito de punta a punta. Checklist físico/de red también cerrado. **Pendiente**: validar velocidad/salto entre waypoints consecutivos (más complejo, único ítem de seguridad que queda abierto).                                            |
| 1 — Investigación: CGA                             | #bloque/en-progreso | Documento de traducción cartesiano→CGA ya escrito (`docs/algebra_geometrica_conforme.md`). Decisión clave: PoE y CGA son bounded contexts separados (ver [[Decisiones de Diseño Clave]]). El resto (fundamentos, `pygafro`, `ga_adapter.py`) sigue pendiente — pospuesto explícitamente, no bloquea nada.                                                                                                                                                        |
| 2 — Investigación: estado del arte                 | #bloque/pendiente   | Ningún ítem empezado. En paralelo, no bloquea nada.                                                                                                                                                                                                                                                                                                                                                                                                              |
| 3 — Percepción y grounding                         | #bloque/en-progreso | Núcleo real hecho: `PerceptionPort`, `Scene` como dicts + `merge`, `perception_node` real, `Commander.follow_perception`, `FilePerceptionAdapter`, `PseudoPerceptionAdapter`. **Pendiente**: `controller_node` aún no cachea `Scene` para su propia planificación, adaptador de percepción CoppeliaSim (ground truth), traducción a CGA, grounding de lenguaje natural.                                                                                          |
| 4 — Planificador reactivo con evitación            | #bloque/en-progreso | `ObstacleAvoidingPlanningAdapter` y `WholeBodyObstacleAvoidingPlanningAdapter` mergeados — son heurísticas geométricas deterministas, no IA. **Pendiente, prioritario tras cerrar Bloque 0**: CHOMP/RRT de verdad (primer hito real de IA/búsqueda), replanificación local, métricas comparables.                                                                                                                                                                |
| 5 — Selección y conmutación de planificador        | #bloque/pendiente   | Nada empezado. Depende del Bloque 4.                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 6 — API de tools para el LLM (Régimen 1)           | #bloque/pendiente   | Solo diseño conceptual en el ROADMAP (nota de diseño: tool-calling, no generación de código libre). Depende de 3 y 4/5.                                                                                                                                                                                                                                                                                                                                          |
| 7 — Supervisión LLM lenta (Régimen 2 lento)        | #bloque/pendiente   | Nada empezado. Depende del Bloque 6.                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 8 — Física real y consolidación                    | #bloque/pendiente   | Nada empezado. Depende del Bloque 6, va al final.                                                                                                                                                                                                                                                                                                                                                                                                                |
| 9 — Generalizar CR5 fijo → robot/escena arbitrario | #bloque/en-progreso | Infraestructura URDF ya integrada: `RobotDescription`, `urdf_kit`, `ControlSession` deriva `joint_names` del URDF, `coppeliasim_scene_builder` generalizado, registro `str→factoría` en ambos `_build_adapter`. **Pendiente**: `PoeKinematicsAdapter` sigue con `_JOINT_ORIGINS`/Jacobiano 6×6 fijo, `dh_adapter`/`ga_adapter` sin tabla/URDF genérico, `controller_node` sin parámetro "qué robot", nombres de escena hardcodeados en `coppeliasim_ik_adapter`. |
| 10 — Dinámica de la cadena cinemática              | #bloque/pendiente   | Añadido 02/09. Ni siquiera decidida la formulación (Newton-Euler vs Lagrangiana). Tensión real sin resolver: hoy se fuerza modo cinemático puro en CoppeliaSim para que la física no interfiera.                                                                                                                                                                                                                                                                 |
| 11 — Controladores de bajo nivel (C/C++)           | #bloque/pendiente   | Añadido 02/09. Pregunta abierta sin responder: si este objetivo de la beca aplica a este repo o a otra pieza de la formación (el CR5 ya trae su propio controlador de fábrica).                                                                                                                                                                                                                                                                                  |
| 12 — Colaboración humano-robot                     | #bloque/pendiente   | Añadido 02/09. Solo preguntas de alcance planteadas, sin decisión. Requisito de seguridad mínimo identificado: tratar a un humano en el espacio de trabajo como obstáculo.                                                                                                                                                                                                                                                                                       |

## Últimos hitos reales (por commit)

- `e009321` (07/09) — Protocolo TCP/IP real contra el CR5 físico + fix de proceso zombie en `ControlSession.stop()` — incluye también, aunque no en el mensaje del commit, los límites articulares, `cp`/suavizado, manejo de errores en `robot_node` y el fix de timing de cierre de sesión, todos verificados en vivo contra el robot físico ese mismo día (ver la nota de arriba).
- `f29bd55` (03/09) — Merge pseudo-perceptor + configuración declarativa de nodos
- `b153fc5` (03/09) — `ControlSession` deriva `joint_names` de un URDF
- `bbe6458` (02/09) — `perception_node` real + `Commander` ensambla objetivos
- `a29d8aa` (01/09) — Merge planificador de evitación de obstáculos
- `73098de` / `dffcb7f` (31/08 y 20/08) — `geometry_kernel` + `RobotDescription`/`urdf_kit` extraídos

## Ver también

- [[Decisiones de Diseño Clave]]
- [[Arquitectura Hexagonal]]
