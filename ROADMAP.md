# Roadmap: hacia un backend de "descripción → nodo robótico validado"

Objetivo de fondo: que este repositorio pueda soportar el flujo descrito en
la conversación de origen — un LLM que, a partir de una descripción textual
y una escena, **compone y genera un nodo (Régimen 1, offline)** con
percepción + planificador con evitación + control clásico; ese nodo corre
con **replanificación reactiva local (Régimen 2 rápido)**; y un LLM
supervisa a ritmo lento, interviniendo solo cuando la tarea se rompe a un
nivel que el planificador no puede resolver solo (Régimen 2 lento).

Estado de partida (ver `README.md`): arquitectura hexagonal con
`RobotControllerPort`/`KinematicsPort` en `shared_kernel`, `ControlSession`
como orquestador de procesos ROS2, adaptadores GA/PoE/DH aún como stubs
(`NotImplementedError`). No hay todavía percepción, planificación con
evitación de obstáculos, ni LLM en el sistema.

**Nota de diseño (revisión sobre el planteamiento original):** el Régimen 1
no genera código libre que luego se ejecuta — el LLM actúa como
**consumidor de una API/tools que expone este propio backend**
(arquitectura tool-providing / function-calling, no Code-as-Policies
literal). El LLM elige y encadena llamadas a operaciones ya validadas del
repo (crear sesión, elegir estrategia, definir obstáculo...); nunca escribe
el código que las implementa. Esto acota mucho más la superficie de
validación del Bloque 6 y hace el sistema más entendible: lo que el LLM
puede hacer está limitado, por diseño, a lo que la API expone.

Orden sugerido: Bloques 0–2 en paralelo desde ya (0 es código, 1–2 son
lectura, no bloquean nada). 3 y 4 pueden avanzar en paralelo una vez cerrado
el 0. 5 depende de 4. 6 depende de 3+4/5. 7 depende de 6. 8 al final, aunque
la parte de seguridad conviene tenerla en mente desde el 0. El Bloque 9
(generalizar de CR5 fijo a escena/robot arbitrario) no bloquea a ningún
otro -- todo lo demás funciona hoy asumiendo un único robot fijo -- pero
cuanto más se implemente sobre el 0/3/4 sin tenerlo en cuenta, más sitios
habrá que revisar luego; conviene tenerlo anotado desde ya aunque no se
aborde todavía.

---

## Bloque 0 — Cerrar el esqueleto clásico (fundación)

- [ ] Implementar cinemática real en al menos un `KinematicsPort`
      (`poe_adapter.py` o `dh_adapter.py`) — hoy `naive_test` y
      `straight_line` son dobles de pruebas, no resuelven IK de verdad.
- [ ] Decidir qué hacer con `ga_adapter.py`: invertir ya en compilar
      `pygafro`, o aparcarlo explícitamente detrás de PoE/DH para no
      bloquear el resto del roadmap con una dependencia externa difícil.
- [ ] Tests de integración end-to-end de `ControlSession` en simulación
      (hoy solo hay el demo manual del README).
- [ ] Decisión mínima sobre `Cr5RealRobotAdapter`/`ros1_kit` (bridge ROS1
      vs reimplementar TCP/IP) — no hace falta implementarlo ya, pero sí
      no dejarlo indefinido para siempre.

## Bloque 1 — Investigación: álgebra geométrica conforme (CGA)

- [ ] Fundamentos de CGA: producto geométrico, blades, cómo un
      plano/esfera/punto se representan como objetos algebraicos (no como
      ecuaciones sueltas).
- [ ] Leer el paper de Löw/Abbet/Calinon que sustenta `gafro` (ya
      referenciado en el propio código) — entender por qué CGA simplifica
      cinemática de cadenas seriales frente a DH.
- [ ] Evaluar `pygafro`/`gafro_ros`: qué API exponen realmente, qué falta
      compilar, si merece la pena para el CR5 concreto.
- [x] Documento corto (para ti, no para nadie más) que traduzca: "plano de
      la mesa" → primitiva CGA, "objeto a evitar" → esfera/región CGA.
      Esto es lo que necesitará el Bloque 3. Ver
      `docs/algebra_geometrica_conforme.md` — extraído de *Geometric
      Algebra for Computer Science* (Dorst/Fontijne/Mann), incluye además
      cinemática directa/inversa en CGA (Bloque 0) y ajuste de esfera a
      puntos (Bloque 3).

## Bloque 2 — Investigación: estado del arte (en paralelo al resto)

- [ ] Revisión de literatura seria, no solo dos búsquedas: partir del
      survey de ML+sampling-based planning y el de language-conditioned
      manipulation (arXiv 2312.10807).
- [ ] Leer a fondo *Code as Policies* — sigue siendo la referencia del
      Régimen 1 en cuanto a validación y composición, aunque aquí se opte
      por tool-calling en vez de generación de código libre (ver nota de
      diseño arriba).
- [ ] Comparar explícitamente los dos paradigmas de acción del LLM:
      *code generation* (Code as Policies) vs *tool-calling/function
      calling* (ReAct, Toolformer, MCP) — entender qué se pierde
      (flexibilidad de composición) y qué se gana (validación, superficie
      acotada, comprensibilidad) al elegir el segundo.
- [ ] Investigar Model Context Protocol (MCP) como mecanismo concreto de
      exposición de tools desde este repo hacia un LLM cliente — es la vía
      más estándar hoy para "tool providing" real.
- [ ] Leer *HyperPlan* y el paper de selección de planificador por
      features de entorno — es la base del Bloque 5.
- [ ] Leer *FaSTrack* (conmutación segura rápido/lento) y *Learning When
      to Quit* — meta-razonamiento sobre cuándo parar de planificar/cuándo
      escalar.
- [ ] Leer *MOPS* — el cruce más cercano a tu idea (LLM + optimización de
      trayectoria).
- [ ] Revisar OMPL: `Syclop` y `CForest` como infraestructura ya existente
      de meta-planificación, para no reconstruirla.
- [ ] Anotar, para cada paper, a qué bloque de este roadmap alimenta —
      evita que la lectura quede desconectada de la implementación.

## Bloque 3 — Percepción y grounding (el cuello de botella real)

- [ ] Nuevo puerto `PerceptionPort` en `shared_kernel` (protocolo, igual
      que `KinematicsPort`): "detecta plano X", "lista obstáculos
      actuales", desacoplado de la implementación de visión.
- [ ] Adaptador de percepción para CoppeliaSim primero (ground truth
      simulado vía API de la escena) — evita depender de visión real desde
      el minuto uno.
- [ ] Capa de traducción "detección → primitiva CGA" (se apoya
      directamente en el Bloque 1).
- [ ] Prototipo de grounding: ligar una frase tipo "el objeto sobre este
      plano" a las primitivas detectadas, con manejo explícito de
      distractores y de que el objeto siga reconocido si aparecen otros
      nuevos.
- [ ] Solo cuando lo anterior funcione en simulación: adaptador de
      percepción con cámara real.

## Bloque 4 — Planificador reactivo con evitación (Régimen 2, rápido)

- [ ] Adaptador tipo CHOMP mínimo (gradiente, evita regiones/esferas CGA
      del Bloque 3) como nueva `strategy` de `controller_node` — mismo
      patrón que ya usa `_build_adapter`.
- [ ] Replanificación local cuando cambia el campo de obstáculos, sin
      ningún LLM en el bucle — esto es lo que hace segura la reactividad
      rápida.
- [ ] RRT como alternativa/baseline para comparar con CHOMP en la misma
      escena.
- [ ] Métricas mínimas (tiempo de replanificación, tasa de éxito) para
      poder comparar planificadores objetivamente en el Bloque 5.

## Bloque 5 — Selección y conmutación de planificador

- [ ] Features de escena estilo HyperPlan, pero derivadas de las
      primitivas CGA (ratio de espacio libre, nº de regiones-obstáculo,
      etc.) en vez de heurísticas ad-hoc.
- [ ] Lógica de selección de planificador puramente clásica primero (sin
      LLM): dado el estado de la escena, elegir CHOMP vs RRT vs
      straight_line.
- [ ] Prototipo de conmutación en caliente dentro de una `ControlSession`
      activa (cambiar de planificador a mitad de ejecución, no solo al
      arrancar).
- [ ] Evaluar si aplica algo tipo FaSTrack (cotas de error precomputadas)
      para que la conmutación tenga garantías, no solo heurística.

## Bloque 6 — API de tools expuesta por el repo, consumida por el LLM (Régimen 1)

- [ ] Diseñar la superficie de la API: qué operaciones expone el backend
      como tools de alto nivel (crear `ControlSession`, listar estrategias
      de planificador disponibles, consultar percepción/escena del
      Bloque 3, definir región de obstáculo, arrancar/detener sesión,
      consultar `feedback`).
- [ ] Formalizar cada tool con un schema tipado (parámetros, validación de
      entrada) — esto sustituye a "validar código generado": aquí no hay
      código que auditar, solo invocaciones a funciones ya verificadas del
      propio repo.
- [ ] Elegir el mecanismo concreto de exposición: servidor MCP sobre este
      repo (natural si el LLM cliente es Claude/similar) vs una capa
      REST/gRPC interna — evaluar cuál encaja mejor con el patrón de
      procesos ROS2 ya existente (`ControlSession` lanza subprocesos).
- [ ] Parser descripción → intención + restricciones, apoyado en el
      grounding del Bloque 3 — esto sigue haciendo falta: es lo que decide
      *qué* tools llamar y con qué parámetros.
- [ ] Adaptar el flujo end-to-end: descripción textual + grounding →
      el LLM encadena llamadas a tools → el backend ejecuta cada llamada,
      sin generación de código intermedio.
- [ ] Piloto: frase en lenguaje natural → secuencia de llamadas a tools →
      `ControlSession` configurada y lanzada, en simulación.

## Bloque 7 — Supervisión LLM a ritmo lento (Régimen 2, lento)

- [ ] Canal de comunicación entre el planificador reactivo (Bloque 4/5) y
      un proceso supervisor LLM aparte: qué eventos disparan consulta, a
      qué cadencia.
- [ ] Política explícita de escalado (qué resuelve el planificador solo
      vs qué sube al LLM) como un puerto/decisión de dominio, no como
      código disperso.
- [ ] El supervisor interviene también a través de la misma API de tools
      del Bloque 6 (p. ej. `cambiar_planificador`, `abortar_sesion`,
      `redefinir_objetivo`) — nunca generando código nuevo, coherente con
      el Régimen 1.
- [ ] Implementar el bucle lento como nodo ROS2 independiente — coherente
      con el patrón de procesos separados que ya usa `ControlSession`.

## Bloque 8 — Física real y consolidación

- [ ] `Cr5RealRobotAdapter` real, con los límites de seguridad reforzados
      antes de ejecutar ahí nada generado por LLM.
- [ ] Desplegar `controller_node` (y/o `robot_node`) en una máquina
      embebida física del robot, separada de donde corre `Commander`. La
      comunicación por topics ya es transparente a la red (ROS2/DDS no
      distingue proceso local de remoto — ver `docs/nodos_ros2.md` §1), lo
      que falta es el lanzamiento: `ControlSession.start()` hoy solo sabe
      arrancar procesos locales vía `subprocess.Popen`. Dos vías: (a)
      lanzamiento remoto (SSH o similar) desde `ControlSession`, o (b) la
      máquina embebida arranca su propio stack (systemd/`robot_upstart`) y
      `Commander` solo se conecta por discovery de DDS sin lanzarlo él —
      esta segunda opción encaja mejor con que `Commander` no debe saber
      nada de cómo se despliega físicamente el sistema.
- [ ] Documentar qué demostró el prototipo, dónde se rompió (grounding,
      validación, frontera rápido/lento) — es el material con el que se
      arma una propuesta de tesis seria.

## Bloque 9 — Generalizar de CR5 fijo a carga de escena/robot arbitrario

Todo el sistema hoy asume un único robot fijo (el CR5) en varios sitios
distintos, sin un único lugar que lo describa — mismo problema repetido:
`["joint1"..."joint6"]` y nombres de escena (`Link6_visual`,
`base_link_respondable`) aparecen copiados a mano en 3-4 archivos en vez de
derivarse de una sola fuente. Este bloque no bloquea a los demás (ver nota
de orden sugerido arriba) pero conviene resolverlo antes de que Bloque 3+
(percepción/planificación) añada más sitios que asuman el mismo robot.

- [ ] **Falta un "descriptor de robot"** — ni `shared_kernel` ni
      `geometry_kernel` tienen un value object que agrupe joint_names +
      base/tip + parámetros cinemáticos (twists/tabla DH) de un robot
      concreto. Hoy esa información vive repartida y repetida a mano en
      `commander_node.py`, `robot_node/node.py` y los adaptadores de
      `controller_node`. Sin esto, cada punto de abajo es un parche local
      en vez de una solución.
- [ ] `poe_adapter.py`: `_JOINT_ORIGINS` (línea ~64, ya anotado con TODO
      inline) y `_JOINT_NAMES` están hardcodeados para el CR5; además el
      tamaño fijo 6×6 de `_jacobian_space`/`_adjoint` asume exactamente 6
      articulaciones. Generalizar de verdad implica parsear el `.urdf`
      (p. ej. `urdf_parser_py`) para derivar twists + nombres + nº de DOF
      en tiempo de carga, no solo mover la tabla a un archivo de config.
- [ ] `dh_adapter.py`: mismo problema con la tabla DH — su TODO ya dice
      "extraer la tabla DH del CR5"; falta que ese TODO contemple explí-
      citamente que la tabla debe poder cambiar por robot, no solo
      completarse una vez para el CR5.
- [ ] `ga_adapter.py`: cuando se resuelva la compilación de
      `pygafro`/`gafro_ros` (Bloque 1), comprobar si `gafro` ya sabe cargar
      un URDF genérico directamente — si es así, este adaptador podría
      generalizarse "gratis" y sería el primero en no necesitar este
      bloque.
- [ ] `controller_node/node.py::_build_adapter` construye todos los
      adaptadores sin argumentos (`PoeKinematicsAdapter()`,
      `CoppeliaSimIkKinematicsAdapter()`) — aunque los adaptadores se
      generalicen, `controller_node` no tiene hoy ningún parámetro ROS2
      para recibir "qué robot" usar.
- [ ] `commander/control_session.py::start()` — confirma el punto anterior:
      `joint_names`/`tip_name`/`scene_path` se reenvían como parámetros
      `-p` solo a `robot_node`, nunca a `controller_node`. Es el hueco de
      cableado concreto que hay que cerrar antes de que el punto anterior
      tenga sentido.
- [ ] `coppeliasim_ik_adapter.py`: `base_name="base_link_respondable"` y
      `tip_name="Link6_visual"` son nombres de objetos de la escena
      `cr5_base.ttt` puestos como default del constructor — y, por el
      punto anterior, hoy no hay forma de override por sesión.
- [ ] `robot_node/node.py::_build_adapter`: la rama `robot_target=="real"`
      instancia `Cr5RealRobotAdapter(host="192.168.1.100", port=29999)`
      hardcodeado, sin parámetros. Añadir un segundo robot físico hoy
      significa una rama `elif` nueva a mano, no configuración. La
      solución general apunta a una fábrica/registro de
      `RobotControllerPort` por identidad de robot, en vez de un
      if/elif fijo en el nodo.
- [ ] `commander_node.py::main()` (demo) y su comentario sobre que
      `cr5_base.ttt` "no trae un dummy tip dedicado" — una vez exista
      carga de escena general, esa clase de suposición (qué objeto sirve
      de tip si la escena no lo declara explícitamente) tiene que
      resolverse por convención documentada o por manifest, no caso a
      caso como ahora.
