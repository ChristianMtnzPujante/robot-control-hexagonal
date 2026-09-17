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
| 1 — Investigación: CGA                             | #bloque/en-progreso | Documento de traducción cartesiano→CGA ya escrito (`docs/algebra_geometrica_conforme.md`). Decisión clave: PoE y CGA son bounded contexts separados (ver [[Decisiones de Diseño Clave]]). **17/09**: F1.1 cerrada (`pygafro` es una rueda de PyPI, nada que compilar) y `ga_adapter.py` ya es real ([[GaKinematicsAdapter]], FK/IK verificadas contra PoE y CoppeliaSim). Pendiente: fundamentos/lectura (Dorst 13–14, paper GAFRO) y la escena conforme (Fase 4a).                                                                                                                                                        |
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

## Detalle por bloque — qué nota del vault toca cada uno

Para cada bloque con algo implementado, las notas donde vive el código/la
decisión real (arquitectura, adaptador, prueba o diario) — no repite el
resumen de la tabla, apunta a donde está el detalle. Los bloques sin
ninguna nota listada no tienen todavía nada implementado en el repo (el
diseño pendiente, si lo hay, vive solo en `ROADMAP.md`).

### Bloque 0 — CR5: instanciación y caso de uso real

- [[Cr5RealRobotAdapter]] — el adaptador real, límites articulares, `cp`,
  reconexión/reintento.
- [[_cr5_protocol (protocolo TCP del CR5)]] — el protocolo TCP en sí,
  función por función.
- [[RobotConnectorPort]] — el puerto que implementa.
- [[PoeKinematicsAdapter]] — la IK real (PoE) que se validó contra el
  robot físico el 07/09.
- [[Commander y ControlSession]] — fix de proceso zombie y de cierre
  prematuro de sesión, ambos encontrados en pruebas reales contra el CR5.
- [[Decisiones de Diseño Clave]] — "CR5 físico: TCP/IP directo, no puente
  ROS1", "config vs. constante" (límites articulares/`cp`).
- [[Conectar un Robot Nuevo]] — el guion genérico, instanciado para el CR5.
- [[Scripts de Demostración]] — sección "PoE contra el CR5 físico" y
  "Primer contacto con hardware".
- [[2026-09-04]] / [[2026-09-07]] — orden cronológico de las pruebas reales.

### Bloque 1 — Investigación: CGA

- [[Primitivas Geométricas]] — por qué `geometry_kernel` NO se
  reinterpreta con multivectores cuando llegue CGA (bounded contexts
  separados).
- [[Decisiones de Diseño Clave]] — la misma decisión, con fecha y motivo.
- [[GaKinematicsAdapter]] — **real desde el 17/09** (F1.2): CGA vía
  `pygafro` (rueda de PyPI, sin compilar nada), FK/IK verificadas contra
  PoE y contra el propio CoppeliaSim (`docs/comparativa_poe_vs_gafro_coppeliasim.md`).
  La prueba de viabilidad F1.1 que lo desbloqueó vive en
  `~/Desktop/doctorado/informe_F1_1_viabilidad_pygafro.md`.

### Bloque 2 — Investigación: estado del arte

Nada implementado ni documentado en el vault todavía.

### Bloque 3 — Percepción y grounding

- [[PerceptionPort]] — el puerto y sus tres adaptadores reales.
- [[Scene y Percepción]] / [[Primitivas Geométricas]] — el agregado
  `Scene` y las primitivas que percepción produce.
- [[FilePerceptionAdapter]] / [[PseudoPerceptionAdapter]] — los dos
  adaptadores dinámicos (releen fichero / inyección programática).
- [[PerceptionNode]] — el nodo ROS2 real, incluido el cableado de
  `perception_target="pseudo"` (08/09).
- [[Infraestructura ROS2 (ros2_kit)]] — serialización de `Scene` y de
  reportes individuales de obstáculo/objeto.
- [[Commander (referencia de código)]] — `follow_perception`, cómo
  `Commander` ensambla la `Scene` completa.
- [[Scripts de Demostración]] — sección "Percepción y replanificación".
- [[2026-09-08]] — cableado de `PseudoPerceptionAdapter` en `perception_node`.

### Bloque 4 — Planificador reactivo con evitación

- [[PlanningPort]] — el puerto y sus dos adaptadores reales.
- [[ObstacleAvoidingPlanningAdapter]] / [[WholeBodyObstacleAvoidingPlanningAdapter]]
  — evitación tip-only vs. cuerpo completo.
- [[_segment_geometry (geometría compartida)]] — la geometría pura que
  comparten ambos.
- [[Evitación de Colisiones]] — prueba real en vivo contra CoppeliaSim,
  comparando los dos planificadores.
- [[Scripts de Demostración]] — sección "Evitación de obstáculos".

### Bloque 5 — Selección y conmutación de planificador

- [[PlannerSelectionPort]] — el puerto ya existe (`FixedPlannerSelectionAdapter`
  cierra el contrato), pero sin ninguna lógica de selección real todavía.

### Bloque 6 — API de tools para el LLM (Régimen 1)

- [[PseudoPerceptionAdapter]] — `description`, la auto-descripción al
  estilo schema de tool de MCP, ya escrita a la espera de que algo la
  consuma.

### Bloque 7 — Supervisión LLM lenta (Régimen 2 lento)

Nada implementado ni documentado en el vault todavía.

### Bloque 8 — Física real y consolidación

Nada implementado ni documentado en el vault todavía.

### Bloque 9 — Generalizar CR5 fijo → robot/escena arbitrario

- [[urdf_kit y RobotDescription]] — `parse_urdf_file`/`RobotDescription`,
  incluido el bug de los `fixed` tras el tip encontrado con el Panda.
- [[Value Objects y Dominio]] — `InvalidRobotDescriptionError`, mismo
  canal `Either` que el resto del dominio.
- [[Anatomía de un Nodo]] §5 — caso real de `joint_names` derivado de un
  URDF, resuelto en `ControlSession` antes de lanzar procesos.
- [[Commander (referencia de código)]] — `ControlSession._resolve_joint_names`.
- [[Conectar un Robot Nuevo]] — el guion genérico que este bloque hace
  posible.
- [[CR5 vs Panda (Generalización)]] — puesta a prueba real contra un
  robot de 7 GDL.
- [[Herramientas de CoppeliaSim]] — `build_scene`, la generalización de
  `build_cr5_scene` encontrada en esa misma prueba.
- [[RobotNode]] / [[ControllerNode]] — el registro `_TARGETS` que permite
  añadir un robot sin tocar el nodo.
- [[CoppeliaSimIkKinematicsAdapter]] — lo que NO generalizó todavía
  (nombres de escena hardcodeados).

### Bloque 10 — Dinámica de la cadena cinemática

Nada implementado ni documentado en el vault todavía.

### Bloque 11 — Controladores de bajo nivel (C/C++)

Nada implementado ni documentado en el vault todavía.

### Bloque 12 — Colaboración humano-robot

Nada implementado ni documentado en el vault todavía.

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
