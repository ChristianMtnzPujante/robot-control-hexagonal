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

**Bloques 10–12 (añadidos el 02/09, tras contrastar este ROADMAP contra los
objetivos formales de la beca):** cubren tres huecos reales que ningún
bloque anterior tocaba -- dinámica del brazo (10), control de bajo nivel
(11) y colaboración humano-robot (12). No bloquean nada de lo anterior ni
dependen de ello salvo donde se indica explícitamente en cada bloque; se
numeran al final para no romper las referencias cruzadas ya existentes a
"Bloque N" en código/docs, no porque sean menos prioritarios -- de hecho,
el 10 y el 12 cubren objetivos formales de la beca hoy sin ningún bloque
propio.

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
- [ ] **Propuesta (03/09, sin diseñar todavía — ver Vikunja):** modularizar
      la definición de canales ROS2 (topics, QoS, tipo de mensaje, quién
      publica/suscribe) fuera del código Python, en JSON/YAML — hoy
      `/perception/scene` está repetido como literal en dos paquetes,
      `GOAL_QOS`/`SCENE_QOS`/`STRATEGY_QOS` son objetos Python en
      `ros2_kit/qos.py`, y `docs/nodos_ros2.md` documenta a mano lo que
      podría derivarse de un solo fichero de datos. Abierto: por-paquete o
      global; sustituye o complementa `docs/nodos_ros2.md`; cómo no perder
      la trazabilidad "por qué esta QoS" que hoy vive en comentarios junto
      a cada valor.

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
- [ ] Decisión de diseño (discutida en la rama de experimentación
      planificador-evita-obstaculo, ver también la tarea de Bloque 4 sobre
      geometría del robot completo): PoE y CGA son bounded contexts
      distintos, cada uno con su propio lenguaje geométrico — cuando GA
      aterrice, `Scene`/las primitivas de `geometry_kernel` NO se
      reinterpretan por debajo con multivectores (revisado; `Scene` y
      `primitives.py` ya no dicen esto). En su lugar, definir una `Scene`
      conforme aparte, con sus propios tipos (multivectores), y la tabla de
      traducción cartesiano→CGA ya documentada en
      `docs/algebra_geometrica_conforme.md` §2 como el punto único donde se
      traduce explícitamente entre ambas — cada `KinematicsPort`/
      `PlanningPort` consume la representación de su propia álgebra, no una
      forma neutra forzada entre las dos.

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

- [x] Nuevo puerto `PerceptionPort` en `shared_kernel` (protocolo, igual
      que `KinematicsPort`): "detecta plano X", "lista obstáculos
      actuales", desacoplado de la implementación de visión. Ya
      implementado (`StaticPerceptionAdapter`, `perception_node`) desde el
      31/08 — quedó sin marcar hasta ahora.
- [ ] **Spike corto, acotado a una sola pregunta:** cuando el
      pseudo-perceptor de abajo "detecta" algo nuevo, ¿vive DENTRO de una
      `ControlSession` ya arrancada (mismo namespace, mismo ciclo de vida
      que `robot_node`/`controller_node` -- procesos reales via
      `subprocess.Popen`, ver `control_session.py`) o es un proceso/nodo
      aparte, de vida propia, al que `Commander` se limita a escuchar?
      ¿Debe coincidir su ciclo de vida con el de la sesión a la que sirve,
      o puede sobrevivirla? Investigación previa al pseudo-perceptor, no
      implementación -- responderla primero condiciona cómo se cablea lo
      demás.
- [ ] **Pseudo-perceptor** (paso previo, más simple, a "ground truth de
      CoppeliaSim" de abajo): un `PerceptionPort` que, a diferencia de
      `StaticPerceptionAdapter` (fijo desde construcción), permita
      "inyectar" eventos con el tiempo — un nuevo obstáculo "detectado", un
      "objetivo" nuevo a seguir — sin cámara ni visión real todavía,
      puramente programático. Primera fase: procesado desde `Commander` (o
      un demo que haga sus veces) — cuando llega un evento nuevo, se
      actualiza la `Scene` y se manda una orden en consecuencia (recalcular
      con `WholeBodyObstacleAvoidingPlanningAdapter`/
      `ObstacleAvoidingPlanningAdapter` de la rama de experimentación, y
      reenviar waypoints). Es la versión más mínima posible de
      "Replanificación local cuando cambia el campo de obstáculos" (Bloque
      4, todavía pendiente) — aquí el "cambio" lo dispara código, no un
      sensor real.
- [ ] **Decisión de diseño, de cara al futuro (Bloque 6 — LLM vía tools):**
      cualquier adaptador de `PerceptionPort` (empezando por el
      pseudo-perceptor de arriba) debería, al configurarse, ANUNCIAR qué
      tipo de información envía junto con una descripción — igual que una
      tool de MCP declara su schema y su descripción — para que un futuro
      LLM pueda descubrir qué perceptores hay disponibles y qué reportan
      sin tener que leer el código. En esta fase no hace falta que nada lo
      consuma todavía (no hay LLM en el bucle) — pero el desarrollo simple
      (el pseudo-perceptor) debería nacer ya con ese metadato (nombre +
      descripción + forma del dato que reporta) para no tener que
      retrofit-earlo cuando llegue el Bloque 6. Ver la tarea "Diseñar
      superficie de la API de tools" de ese bloque.
- [x] `Scene.obstacles` pasa de `List[SphereObstacle]` a `Dict[str,
      SphereObstacle]`, y `Scene` gana `merge(other)` — decisión tomada
      al diseñar `perception_node` (conversación 02/09): con obstáculos
      identificados por nombre, un productor puede releer su fuente
      entera en cada ciclo sin llevar diff (sobrescribe por clave, igual
      que ya hacían `planes`/`objects`), y varias `Scene` parciales (una
      por perceptor) se combinan clave a clave en una completa vía
      `merge`. Actualizados todos los consumidores que trataban
      `Scene.obstacles` como lista (`ObstacleAvoidingPlanningAdapter`,
      `WholeBodyObstacleAvoidingPlanningAdapter`,
      `coppeliasim_scene_builder`, demos y tests).
- [x] Decisión de diseño: **`Commander` ensambla la `Scene` completa**
      (vía `Scene.merge`) a partir de las piezas que le reporten uno o
      más perceptores, y reenvía el resultado a cada `ControlSession` que
      lo necesite — no `controller_node` escuchando directo a un único
      productor, que era el boceto anterior de `docs/nodos_ros2.md` §4
      (pendiente de actualizar ese documento). Encaja con el spike de
      ciclo de vida ya resuelto (Vikunja #89): el/los perceptor(es) tienen
      vida propia, fuera de cualquier `ControlSession`, y `Commander` es
      quien ya cruza esa frontera (crea namespaces, publica `<ns>/goal`).
      No viola su invariante de no saber de estrategias/backends: ensamblar
      `Scene` es plumbing de datos (fusión de dicts), no una decisión de
      planificación.
- [x] Formato del topic `/perception/scene` decidido e implementado: JSON
      en `std_msgs/String` (mismo patrón que `<ns>/feedback`, sin paquete
      de interfaces nuevo) — `to_scene_msg`/`from_scene_msg` en
      `ros2_kit/messages.py`, con `SCENE_QOS` nuevo (RELIABLE +
      TRANSIENT_LOCAL, mismo motivo que `GOAL_QOS`). Tests de round-trip
      en `ros2_kit/test/test_messages.py`.
- [x] **`perception_node/node.py`**: primer nodo ROS2 real de percepción
      — publica periódicamente (`scene_publish_period_seconds`) lo que su
      adaptador (`perception_target`: `fichero`/`estatico`) reporte, en
      `/perception/scene` (topic GLOBAL, sin namespace de sesión — vive
      fuera de cualquier `ControlSession`, ver spike #89).
- [x] **`Commander.follow_perception(session_name)`**: se suscribe a
      `/perception/scene` y reenvía como `send_goal` cualquier objetivo
      NUEVO en `Scene.objects["objetivo"]` (dedupe por `Point`, para no
      remandar el mismo goal en cada ciclo del publisher). `controller_node`
      no necesitó ningún cambio — `_on_goal` ya calculaba trayectoria
      nueva desde la configuración actual y se queda "esperando" (sin
      `_pending_waypoints`) entre goals. `FilePerceptionAdapter` gana la
      sintaxis `objetivo x y z` (clave fija, distinta de un obstáculo por
      número de campos) para poder disparar esto desde un fichero.
      Verificado de punta a punta (fichero → adaptador → mensaje → Commander
      → `Pose`). Demo: `commander/file_perception_goal_demo.py`
      (`ros2 run commander file_perception_goal_demo`).
- [ ] Pendiente (alcance original, no cubierto por lo anterior):
      `controller_node` sigue sin recibir/cachear `Scene` para SU PROPIA
      planificación (evitar obstáculos) — la tubería de arriba solo
      reenvía el objetivo como `Pose`, no la `Scene` completa. Tampoco se
      ha ejercitado `Scene.merge` con más de un perceptor escuchado a la
      vez (`follow_perception` solo suscribe un topic).
- [x] **`FilePerceptionAdapter`** (`perception_node/adapters/`): tercer
      adaptador de `PerceptionPort`, banco de pruebas mínimo de
      percepción sin depender de CoppeliaSim — relee un fichero de texto
      ENTERO en cada `get_scene()` (una línea por obstáculo, `nombre x y
      z radio`, o `objetivo x y z` para el objetivo; comentarios con `#`),
      sin diff ni estado interno más allá de la ruta. Ejemplo en
      `perception_node/example_obstacles.txt`. Tests en
      `perception_node/test/test_file_perception_adapter.py`.
- [ ] Adaptador de percepción para CoppeliaSim primero (ground truth
      simulado vía API de la escena) — evita depender de visión real desde
      el minuto uno. Punto sin resolver, no cosmético: no hay precedente
      en el repo de leer el RADIO de una esfera vía la API ZMQ
      (`createPrimitiveShape` no lo expone como propiedad; hace falta
      `getShapeBB`/bounding box) — verificar contra CoppeliaSim real antes
      de darlo por hecho. `FilePerceptionAdapter` de arriba permite probar
      el resto de la tubería sin esperar a resolver esto.
- [ ] Capa de traducción "detección → primitiva CGA" (se apoya
      directamente en el Bloque 1).
- [ ] Prototipo de grounding: ligar una frase tipo "el objeto sobre este
      plano" a las primitivas detectadas, con manejo explícito de
      distractores y de que el objeto siga reconocido si aparecen otros
      nuevos.
- [ ] Solo cuando lo anterior funcione en simulación: adaptador de
      percepción con cámara real.

## Bloque 4 — Planificador reactivo con evitación (Régimen 2, rápido)

- [ ] Geometría del robot completo, no solo el tip: hoy la evitación de
      obstáculos (incluida `ObstacleAvoidingPlanningAdapter`, rama de
      experimentación) solo comprueba la trayectoria de un punto — el
      efector — pero un robot real puede colisionar con cualquier eslabón,
      no solo con la punta. Hace falta poder consultar, para una
      `JointConfiguration` dada, la pose de CADA articulación/eslabón, no
      solo la del tip: `PoeKinematicsAdapter` ya acumula internamente las
      transformadas intermedias por articulación para llegar a la del tip
      (`_forward_kinematics`, ver `poe_adapter.py`) — falta exponerlas
      todas, no solo la última. Para robots de geometría conocida (como el
      CR5, vía `RobotDescription` — Bloque 9) esto se deriva directamente
      sin percepción; ver también Bloque 1 (CGA): representar cada eslabón
      como una recta podría ser la forma natural de comprobar distancia a
      los `SphereObstacle` de la `Scene` (CGA representa líneas de forma
      nativa — comprobar si `gafro`/`pygafro` ya lo resuelve antes de
      construirlo a mano).
- [ ] Adaptador tipo CHOMP mínimo (gradiente, evita regiones/esferas CGA
      del Bloque 3) como nueva `strategy` de `controller_node` — mismo
      patrón que ya usa `_build_adapter`.
- [ ] Replanificación local cuando cambia el campo de obstáculos, sin
      ningún LLM en el bucle — esto es lo que hace segura la reactividad
      rápida.
- [ ] RRT como alternativa/baseline para comparar con CHOMP en la misma
      escena. Nota de alineación (02/09, contraste contra objetivos de la
      beca): los dos planificadores ya implementados y verificados esta
      semana (`ObstacleAvoidingPlanningAdapter`,
      `WholeBodyObstacleAvoidingPlanningAdapter`) son heurísticas
      geométricas deterministas, no técnicas de IA -- CHOMP/RRT de este
      bloque son, con diferencia, el primer hito real y concreto hacia el
      objetivo de formación en IA de la beca (búsqueda/optimización, no
      solo lectura). Priorizar en cuanto se cierre el Bloque 0.
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
      consultar `feedback`). Incluye la auto-descripción de perceptores ya
      anotada en el Bloque 3: cada `PerceptionPort` debería declarar su
      propio nombre/descripción/forma del dato, al estilo del schema de una
      tool de MCP, para que esta API pueda listarlos sin hardcodear nada.
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

## Bloque 10 — Dinámica de la cadena cinemática

Añadido el 02/09 tras contrastar este ROADMAP contra los objetivos
formales de la beca: "simulación cinemática **y dinámica** de cadenas
robóticas lineales" tiene una mitad, la dinámica, sin ningún bloque
propio hasta ahora — y el trabajo de esta semana, sin querer, ha ido en
dirección contraria (ver la tercera tarea).

- [ ] Decidir formulación: Newton-Euler recursivo sobre el mismo
      formalismo de twists que ya sustenta `poe_adapter.py` (Lynch & Park,
      *Modern Robotics*, cap. 8 — misma fuente que el PoE ya implementado,
      no hace falta una base matemática nueva) frente a una Lagrangiana
      clásica. La primera reutiliza directamente `RobotDescription`/twists
      sin re-derivar nada geométrico.
- [ ] Parámetros dinámicos que faltan por completo hoy: masa e inercia por
      eslabón. Comprobar si el URDF real del CR5
      (`~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/cr5_robot.urdf`)
      trae ya `<inertial>` utilizables, o hay que estimarlos/asumirlos
      para el primer experimento.
- [ ] **Resolver la tensión real con el hallazgo del 01/09:** hoy se
      fuerza `jointmode_kinematic` + `modelproperty_not_dynamic` al
      importar el robot en CoppeliaSim (ver
      `whole_body_obstacle_avoiding_planning_adapter.py`), precisamente
      para que la física NO interfiera con el control por posición. Un
      experimento de dinámica de verdad necesita lo contrario. Decidir si
      conviven como dos modos explícitos (control cinemático puro vs.
      control con dinámica activa, elegido por sesión) o si la dinámica se
      calcula aparte, en software, sin tocar el modo del simulador.
- [ ] Primer experimento concreto: control por par calculado (*computed
      torque control*) sobre el CR5 simulado, comparado contra el control
      puramente cinemático que ya existe — con la física real de
      CoppeliaSim como referencia de validación, mismo patrón que
      `two_sessions_demo.py` ya usa para comparar PoE contra simIK.

## Bloque 11 — Controladores de bajo nivel para servos (C/C++)

Añadido el 02/09, mismo contraste contra la beca. Hueco casi total hoy:
todo el repo es Python/ROS2 a nivel de aplicación.

- [ ] **Pregunta abierta real, sin resolver, antes de planificar nada
      más aquí:** ¿este objetivo de la beca se cubre en
      `robot-control-hexagonal` o en otra pieza de la formación? El CR5
      real ya trae su propio controlador de bajo nivel de fábrica —
      `Cr5RealRobotAdapter` (Bloque 8) hablaría con él por TCP/IP, no lo
      sustituiría ni lo reimplementaría. Si el objetivo de la beca es
      programar servo-control desde cero, este repo (arquitectura de alto
      nivel sobre un robot que ya trae su propio controlador) probablemente
      no es el sitio natural — confirmarlo es el primer paso, no una
      formalidad.
- [ ] Si aplica aquí: identificar una plataforma de práctica desacoplada
      del CR5 (una tarjeta de desarrollo + un servo/motor DC de pruebas)
      para no depender de tener acceso al brazo físico para esta parte.
- [ ] Formación de base antes de nada específico de robótica: bucle de
      control PID en C/C++ sobre microcontrolador.

## Bloque 12 — Colaboración humano-robot

Añadido el 02/09. El objetivo de beca de "generación de trayectorias con
realimentación visual" menciona explícitamente un "sistema
humano-manipulador robótico" — hoy no hay ni una mención a esto en
ningún bloque del ROADMAP, ni siquiera como pregunta abierta.

- [ ] Investigación de alcance, sin decisión tomada todavía: ¿qué
      significa "colaboración humano-robot" en este proyecto en concreto?
      ¿Detección de presencia/intención humana en el espacio de trabajo?
      ¿Parada de seguridad reactiva? ¿Planificación de tareas compartidas
      donde humano y robot se turnan o cooperan en la misma tarea?
- [ ] Requisito de seguridad mínimo, antes de cualquier otra cosa: si un
      humano entra en el espacio de trabajo, el planificador reactivo
      (Bloque 4) debe tratarlo como un obstáculo. Comprobar si esto sale
      "gratis" en cuanto exista percepción real (Bloque 3) tratando a la
      persona como un `SphereObstacle` más, o si necesita lógica propia
      (urgencia/prioridad distinta a un obstáculo estático — p. ej. parada
      inmediata en vez de replanificación con margen).
- [ ] Conectar con el Bloque 7 (Régimen 2 lento): ¿una intervención
      humana debe escalar al supervisor LLM igual que un fallo del
      planificador, o es una tercera vía de escalado con su propia
      política?
