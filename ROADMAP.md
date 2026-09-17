# Roadmap: hacia un backend de "descripción → nodo robótico validado"

Objetivo de fondo: que este repositorio pueda soportar el flujo descrito en
la conversación de origen — un LLM que, a partir de una descripción textual
y una escena, **compone y genera un nodo (Régimen 1, offline)** con
percepción + planificador con evitación + control clásico; ese nodo corre
con **replanificación reactiva local (Régimen 2 rápido)**; y un LLM
supervisa a ritmo lento, interviniendo solo cuando la tarea se rompe a un
nivel que el planificador no puede resolver solo (Régimen 2 lento).

Estado de partida (ver `README.md`): arquitectura hexagonal con
`RobotConnectorPort`/`KinematicsPort` en `shared_kernel`, `ControlSession`
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

**Cruce con la tesis (añadido el 16/09):** cada bloque lleva debajo de su
título una línea `Tesis:` que dice a qué fase y objetivo de la propuesta
(`~/Desktop/doctorado/propuesta_tesis_CGA_LLM.tex`, fases 1–5 y objetivos
H1–H4) alimenta, o si queda fuera de su camino crítico. Los códigos `F1.1`,
`F4a.2`, etc. son los paquetes de trabajo de la hoja de ruta de la tesis;
es la misma idea que ya pide el Bloque 2 para los papers ("anotar a qué
bloque alimenta"), aplicada en sentido contrario.

---

## Bloque 0 — CR5: instanciación y caso de uso real

> **Tesis:** Fase 1 (base de ejecución ya validada en hardware; cierre en
> `F1.8`) y Fase 2 (los adaptadores PoE/DH intercambiables son el "brazo
> convencional" del banco H1.1, `F2.3`). Estado: casi cerrado.

Objetivo de este bloque (redefinido 04/09): dejar de ser la base teórica del
sistema clásico (esa base ya no depende de un robot fijo, ver Bloque 9) y
pasar a ser la instanciación concreta y el ejemplo de uso real de la
arquitectura hexagonal contra el CR5 -- lo que sigue siendo específico de
ESTE robot, no generalizable. `ga_adapter.py` se movió al Bloque 1: no
bloquea esta instanciación, solo se pospone.

- [x] Implementar cinemática real en al menos un `KinematicsPort`
      (`poe_adapter.py` o `dh_adapter.py`) — hoy `naive_test` y
      `straight_line` son dobles de pruebas, no resuelven IK de verdad.
- [ ] Tests de integración end-to-end de `ControlSession` en simulación
      (hoy solo hay el demo manual del README).
- [x] Decisión sobre `Cr5RealRobotAdapter`/`ros1_kit` (04/09): **reimplementar
      TCP/IP directo**, no puente ROS1 — este entorno no tiene ROS1 Noetic
      instalado junto a ROS2 Humble (requisito duro de la vía puente,
      documentado en el propio `bridge.py`), mientras que reimplementar el
      protocolo es puro Python/ROS2, sin dependencias nuevas.

**Guion de conexión real con el CR5 físico (añadido 04/09, ver Vikunja
Bloque 0 #20/#110/#111/#112/#113):**

- [x] Vía "reimplementar TCP/IP" (04/09, **corregida el mismo día**):
      `Cr5RealRobotAdapter` habla el protocolo real, en
      `adapters/_cr5_protocol.py` + `adapters/cr5_real_adapter.py`. La
      primera versión se basó solo en el driver de referencia
      (`dobot_bringup/include/dobot_bringup/commander.h`,
      `~/ros2_ws/src/TCP-IP-ROS-6AXis`, fechado 2021) y asumía un puerto de
      movimiento aparte (30003) con `JointMovJ(j1,...,j6)`. Al encontrar y
      leer el **manual oficial del fabricante**
      (`~/ros2_ws/src/TCP-IP-ROS-6AXis/misc/Dobot TCP_IP二次开发接口文档_V4.6.5_20251015_cn.pdf`,
      2025 — mucho más reciente que el driver) resultó que ese puerto NO
      existe en el protocolo actual: solo hay 29999/30004/30005/30006, y el
      movimiento articular se manda como `MovJ(joint={j1,...,j6})` por el
      MISMO puerto 29999 ("Dashboard"). El formato de la trama real-time
      (puerto 30004, 1440 bytes, `q_actual` en el byte 432, valor mágico en
      el 48) sí coincidía entre driver y manual -- esa parte no cambió.
      Cubierto por tests contra un servidor TCP de mentira que imita el
      protocolo del manual (`src/robot_node/test/test_cr5_protocol.py`,
      `test_cr5_real_adapter.py`). **Verificado contra el robot físico el
      07/09**: el manual coincide con el firmware de este CR5 concreto
      (confirmado moviendo el robot de verdad, no solo leyendo `RobotMode`).
- [x] Vía "puente ROS1" (04/09, descartada): no se implementa — la decisión
      de arriba fue por la vía TCP/IP directo. `ros1_kit/bridge.py` se deja
      tal cual, como boceto sin usar.
- [x] Cablear `robot_node/node.py::_build_adapter` (rama `"real"`) para
      pasar la info de conexión real (04/09): con la vía TCP/IP elegida, los
      2 puertos son constantes del protocolo, no parámetros — lo único que
      faltaba pasar de verdad era el host y los `joint_names` (nuevo
      parámetro ROS2 `cr5_host`, ver `robot_node.yaml`).
- [x] Validar límites articulares en `Cr5RealRobotAdapter.set_joints` antes
      de mandar `MovJ` (07/09, parcial): rechaza sin mandar nada si algún
      ángulo excede ±360° (J1/J2/J4/J5/J6) o ±160° (J3) — límites
      verificados contra TRES fuentes oficiales independientes (manual de
      usuario, manual de hardware, página de producto), coinciden con la
      URDF local. Validación de velocidad/salto entre waypoints
      consecutivos sigue sin resolver (más compleja, ver Vikunja #114).
- [x] Checklist físico/de red antes del primer movimiento real (07/09): IP
      real confirmada (`192.168.5.1`, no el `192.168.1.100` de relleno),
      los 2 puertos TCP (29999, 30004) accesibles, procedimiento de e-stop
      probado — un incidente real durante las pruebas, no solo teórico
      (ver Vikunja #110), zona despejada y movimientos pequeños (3-6cm)
      para las primeras pruebas.
- [x] Primera validación real end-to-end (07/09): el sistema resolvió una
      IK real con PoE (subir/bajar el TCP unos centímetros, con y sin
      girar joint6) y la ejecutó con éxito en el CR5 físico, a través de
      TODO el stack (Commander → ControlSession → controller_node →
      robot_node → Cr5RealRobotAdapter → CR5). Posición final coincidió
      EXACTAMENTE con la predicción de PoE. Objetivo de cierre de este
      bloque, cumplido — varios hallazgos reales por el camino (corte de
      red físico, incidente de e-stop, proceso zombie en
      `ControlSession.stop()`, `MovJ` sin `cp` vibrando, cierre de sesión
      prematuro respecto al robot real, error "-7 script pausado" resuelto
      con un power-cycle del controlador): detalle completo en Vikunja
      #110, #116, #117, #118.
- [x] **(14/09)** Gesto de saludo (`cr5_wave_sim_demo.py` en `commander`,
      `cr5_wave_demo.py` en `robot_node`) — a petición del usuario, tras el
      círculo: la punta recorre un ARCO suave de lado a lado (más alto en
      el centro que en los extremos) mientras la herramienta, inclinada
      HACIA ARRIBA, bascula en fase con el desplazamiento. La segunda
      mitad (arco + inclinación) salió de una corrección del usuario sobre
      la primera versión, que era una línea recta con la herramienta
      horizontal: "tiene una pinza que sería como la mano conectada, así
      quedaría mejor". Por defecto ±0.15m, 0.05m de flecha de arco, ±25°
      de basculación y 25° de inclinación, a lo largo de 3 ciclos de 16
      puntos = 48 `MovJ` seguidos por la misma conexión (más del doble que
      el círculo completo). Verificado en vivo contra CoppeliaSim;
      **pendiente de ejecutar contra el CR5 físico** (requiere
      confirmación interactiva).

      **Hallazgo geométrico 1 (14/09) — por qué la home no vale, ahora con
      un número.** Extiende el hallazgo del 08/09 ("la home no puede subir,
      solo bajar", encontrado por barrido de IK al diseñar el semicírculo).
      En la home el tip está a 0.933m del hombro y el alcance de catálogo
      del CR5 es 0.9m: la home está literalmente en el borde del espacio
      alcanzable, con el brazo completamente extendido en vertical. De ahí
      que tampoco pueda moverse de lado manteniendo la altura — cualquier
      punto con el mismo z y distinto x queda TODAVÍA más lejos. No hace
      falta invocar ninguna singularidad para explicarlo: es alcance puro.
      La postura de saludo por defecto deja el tip a 0.647m del hombro,
      25cm de margen.

      **Hallazgo geométrico 2 (14/09) — el cabeceo por conjugación, no por
      IK.** Inclinar la herramienta hacia arriba es una rotación sobre el
      eje X del mundo, y en la postura de saludo NINGÚN joint del CR5 la
      da suelto: el eje de `joint4`/`joint6` es el eje Y del mundo y el de
      `joint5` es el eje Z. Pero conjugar una rotación sobre Z por ±90°
      sobre Y la convierte en una sobre X — Ry(-90)·Rz(β)·Ry(90) = Rx(-β)
      — así que con `joint4`=+45 y `joint6`=-90 fijos, **`joint5` pasa a
      SER directamente el ángulo de cabeceo** (verificado numéricamente:
      error de orientación ~1e-6 frente al objetivo). Eso deja la postura
      de saludo en (0, -45, 90, 45, β, -90), analítica y sin IK.

      La alternativa obvia —pedir la pose inclinada por IK, rotando la
      pose "plana" sobre X y dejando que Newton-Raphson encuentre la
      postura— se probó y se descartó: converge, pero a una rama
      contorsionada (`joint1`=-9°, `joint6`=-69°) desde la que el barrido
      del saludo daba saltos de **hasta 51° entre waypoints consecutivos**,
      porque la IK iba saltando de rama a lo largo del recorrido. Fijando
      la postura analíticamente y dejando que la IK solo siga deltas
      pequeños, ese máximo baja a ~7°. Es el mismo tipo de lección que ya
      dio `_snap_to_exact_start_if_needed` en el círculo: cuando hay
      redundancia, conviene ANCLAR la rama en vez de confiar en que la IK
      elija siempre la misma.

      Resultado medido con los valores por defecto: `joint1` se queda a 0°
      y `joint5`/`joint6` CLAVADOS en la inclinación durante todo el
      saludo — el gesto entero lo hacen `joint2`/`joint3`/`joint4`, el
      plano del brazo. El robot no gira el "cuerpo" ni retuerce la muñeca:
      solo mece el brazo con la mano fija apuntando hacia arriba.
      De autocolisión es el recorrido MÁS holgado de los cartesianos del
      repo: distancia mínima entre eslabones no adyacentes de 0.116m en
      todo el gesto — la misma que en la home, y muy lejos de los 0.077m a
      los que saltó la alarma [76] del fabricante durante el semicírculo.
      Y cierra sobre su postura de partida por construcción
      (sin(2π·ciclos)=0) con 0.01° de residuo, muy por debajo del umbral
      de 0.5° de `_snap_to_exact_start_if_needed`.

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

> **Tesis:** Fase 1 · Objetivo H2.1 (`F1.1` viabilidad de `pygafro`,
> `F1.2` GAFRO como `KinematicsPort`, o `F1.2b` plan B en Python); Fase 4a ·
> H3.1 (`F4a.2`, escena conforme); Fase 4b · H4.3b (MPC conforme). Es el
> **bloqueador activo** de la tesis: `F1.1` es la primera tarea del sprint.

- [ ] Fundamentos de CGA: producto geométrico, blades, cómo un
      plano/esfera/punto se representan como objetos algebraicos (no como
      ecuaciones sueltas).
- [ ] Leer el paper de Löw/Abbet/Calinon que sustenta `gafro` (ya
      referenciado en el propio código) — entender por qué CGA simplifica
      cinemática de cadenas seriales frente a DH.
- [x] **(17/09)** Evaluar `pygafro`/`gafro_ros`: qué API exponen realmente, qué falta
      compilar, si merece la pena para el CR5 concreto. *Hecho (F1.1, informe en
      `~/Desktop/doctorado/informe_F1_1_viabilidad_pygafro.md`, workspace
      `~/gafro_ws`): `pygafro` de PyPI (1.3.5) no necesita compilarse y da la
      FK del CR5 idéntica a `poe_adapter.py` (error <1e-14, 60× más rápido);
      `gafro_ros` es ROS1; `gafro_ros2` compila contra Humble fijando
      `sackmesser` a 05/2025 + parche de 1 línea, y su `convert_urdf` sí
      convierte el URDF del CR5 a YAML cargable por `pygafro`.*
- [x] **(17/09)** Decidir qué hacer con `ga_adapter.py`: invertir ya en compilar
      `pygafro`, o aparcarlo explícitamente detrás de PoE/DH — no bloquea
      nada (la cinemática real ya se resolvió en Bloque 0 con PoE), es solo
      cuándo invertir en la dependencia externa. *(movido desde Bloque 0 el
      04/09 — GA se pospone, no bloquea la instanciación CR5)* *Decidido:
      invertir ya (F1.2) — la dependencia es una rueda de PyPI, no una
      compilación; `ga_adapter.py` se implementará sobre `pygafro.System`
      construido desde `RobotDescription` (ver `build_cr5_system()` en
      `~/gafro_ws/f1_1/cr5_fk_check.py`).*
- [x] **(17/09)** `GaKinematicsAdapter` real (F1.2): `pygafro.System`
      construido desde el mismo `RobotDescription` que PoE; FK, `link_poses`
      e IK (Newton-Raphson amortiguado sobre el log del motor, mismo esquema
      y tolerancias que PoE). 8 tests cruzados contra PoE. Comparativa en
      CoppeliaSim (`cr5_poe_vs_gafro_sim_demo.py` →
      `docs/comparativa_poe_vs_gafro_coppeliasim.md`): misma pose (FK vs
      CoppeliaSim < 1 µm, IK < 0,1 mm), IK ~3× y FK ~4,6× más rápidas en
      GA. *Hallazgo:* con `joint5=0` (todo el arco) joint2/3/4/6 son
      paralelos → familia continua de soluciones; las dos IK dan soluciones
      articulares distintas (hasta 274 mrad) para la misma pose. Refuerza
      lo del 14/09: anclar la rama en articulaciones, no delegar en la IK.
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

> **Tesis:** Fase 1 · hito (artículo de revisión, `F1.3`) y revisión formal
> de A.2 con protocolo de búsqueda. Las lecturas de este bloque y el plan de
> lectura del proyecto Vikunja "Doctorado — Tesis" son la misma tabla de
> evidencia.

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

> **Tesis:** Fase 2 (generador de escenas del banco H1.1, `F2.2`, sobre
> `FilePerceptionAdapter`/`coppeliasim_scene_builder`); Fase 4a · H3.1/H3.2
> (`F4a.1` ground truth de CoppeliaSim, `F4a.2` traducción detección →
> primitiva CGA, `F4a.3` cierre de la tubería de escena, `F4a.4` percepción
> con DL). Estado: avanzado.

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
- [x] **Pseudo-perceptor** (paso previo, más simple, a "ground truth de
      CoppeliaSim" de abajo): un `PerceptionPort` que, a diferencia de
      `StaticPerceptionAdapter` (fijo desde construcción), permita
      "inyectar" eventos con el tiempo — un nuevo obstáculo "detectado", un
      "objetivo" nuevo a seguir — sin cámara ni visión real todavía,
      puramente programático. Implementado (`PseudoPerceptionAdapter`,
      `perception_node/adapters/`) y mergeado desde la rama
      `experimento/pseudo-perceptor` (`f29bd55`) — casilla sin marcar hasta
      ahora pese a estar hecho. Primera fase: procesado desde `Commander` (o
      un demo que haga sus veces) — cuando llega un evento nuevo, se
      actualiza la `Scene` y se manda una orden en consecuencia (recalcular
      con `WholeBodyObstacleAvoidingPlanningAdapter`/
      `ObstacleAvoidingPlanningAdapter` de la rama de experimentación, y
      reenviar waypoints). Es la versión más mínima posible de
      "Replanificación local cuando cambia el campo de obstáculos" (Bloque
      4, todavía pendiente) — aquí el "cambio" lo dispara código, no un
      sensor real.
      **(08/09)** Hasta hoy solo se usaba desde dentro del mismo proceso
      Python (un demo instanciándolo directamente) — ahora está cableado de
      verdad en `perception_node` (`perception_target="pseudo"`), con dos
      topics de entrada nuevos (`/perception/report_obstacle`,
      `/perception/report_object`, JSON en `std_msgs/String`, mismo patrón
      que `/perception/scene`) para que un proceso externo pueda inyectar
      eventos sin compartir proceso con el nodo. Verificado en vivo con
      `rclpy` real (no solo tests): un obstáculo y un objeto publicados por
      un proceso aparte aparecen en `/perception/scene` sin reiniciar el
      nodo.
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

> **Tesis:** Fase 1 · `F1.6` (los adaptadores de autocolisión y cuerpo
> completo son la comprobación previa a ejecutar de la Protocol Layer,
> verify-then-act); Fase 4a (capa de ejecución del pipeline, `F4a.6`).
> Estado: avanzado. CHOMP/RRT no están en el camino crítico de H1–H4.

- [x] Geometría del robot completo, no solo el tip — resuelto por
      `WholeBodyObstacleAvoidingPlanningAdapter` (mergeado 01/09), que
      consulta `link_poses` para comprobar cada eslabón. Casilla sin
      marcar hasta ahora pese a estar hecho.
      **(08/09)** Además, `forward_kinematics`/`link_poses` pasan de ser
      un método extra de `PoeKinematicsAdapter` a parte FORMAL de
      `KinematicsPort` (`shared_kernel/ports.py`) — antes cada consumidor
      (`ObstacleAvoidingPlanningAdapter`/`WholeBodyObstacleAvoidingPlanningAdapter`)
      declaraba su propio `Protocol` local más estrecho para exigirlos por
      duck typing; ahora viven en el puerto mismo. `CoppeliaSimIkKinematicsAdapter`
      los implementa por primera vez (sobre el mismo entorno IK aislado
      que ya usaba `compute_trajectory`, sin verificar aún en vivo contra
      CoppeliaSim); `GaKinematicsAdapter`/`DhKinematicsAdapter` (stubs) y
      `NaiveTestKinematicsAdapter`/`StraightLineKinematicsAdapter` (dobles
      de test) los declaran lanzando `NotImplementedError`, por
      consistencia con `compute_trajectory`.
      Para robots de geometría conocida (como el
      CR5, vía `RobotDescription` — Bloque 9) esto se deriva directamente
      sin percepción; ver también Bloque 1 (CGA): representar cada eslabón
      como una recta podría ser la forma natural de comprobar distancia a
      los `SphereObstacle` de la `Scene` (CGA representa líneas de forma
      nativa — comprobar si `gafro`/`pygafro` ya lo resuelve antes de
      construirlo a mano).
- [x] **(08/09)** Autocolisión como comprobación de base, no ad-hoc —
      motivado por un hallazgo real: `cr5_semicircle_demo.py` disparó una
      alarma real del CR5 físico (`GetErrorID()`=[76], "el extremo
      interfiere con el cuerpo del robot", nivel 5) al mandar una
      secuencia de `MovJ` consecutivos. Nuevo `SelfCollisionAwarePlanningAdapter`
      (tercer `PlanningPort`): cápsulas (segmento + radio) sobre
      `link_poses`, rechaza la trayectoria entera si algún waypoint
      colisiona consigo mismo — no intenta rodearla (eso es CHOMP/RRT, más
      abajo). Radio único calibrado contra dos puntos reales (home: 11.6cm
      de margen mínimo; la secuencia que disparó la alarma: 7.7cm en el
      peor tramo), no adivinado — ver la vault, Decisiones de Diseño Clave.
      Corregido también `cr5_disable_demo.py`: en alarma, `DisableRobot()`
      no se ejecuta hasta `ClearError()` (manual oficial, sección "Códigos
      de error generales"), paso que faltaba.
      **Segunda vuelta, mismo día**, a petición del usuario ("necesitamos
      generar una trayectoria en la que no colisione, por los mismos
      puntos"): nuevo `SelfCollisionAvoidingPlanningAdapter` (misma
      familia) que, en vez de solo rechazar, explota la singularidad de
      `joint5≈0` (ya documentada en `PoeKinematicsAdapter`) — cerca de ahí
      `joint4`/`joint6` tienen un grado de libertad casi redundante para
      una orientación dada, reintenta la IK del mismo objetivo desde una
      semilla con esos dos joints desplazados en pasos crecientes hasta
      encontrar una rama libre. Verificado contra el incidente real
      completo: 4° de desplazamiento ya basta, salto adicional de ~10°
      (frente a ~180° de una vuelta de muñeca completa) — las 9
      configuraciones del arco real se alcanzan sin excepción, mismo
      destino cartesiano exacto. Wireado en `cr5_semicircle_sim_demo.py`
      (verificado en vivo contra CoppeliaSim) y `cr5_semicircle_demo.py`
      (verificado contra el arnés de servidor TCP de mentira).
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

> **Tesis:** sin mapeo directo a H1–H4. Podría entrar como baseline en el
> banco H1.1 si la comparativa lo pide; no bloquea ninguna fase.

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

> **Tesis:** Fase 1 · Objetivo H2.2 (`F1.4` catálogo de tools con esquema
> CGA, `F1.5` servidor MCP + piloto, `F1.6` Protocol Layer verify-then-act);
> Fase 4a · H4.3a (restricciones discretas como parámetros de tools,
> `F4a.5`). La propuesta ya fija MCP como mecanismo (A.3), lo que cierra la
> tarea "MCP vs REST/gRPC" de abajo. Estado: sin empezar.

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

> **Tesis:** Fase 1 · Objetivo H2.3 (`F1.7`, grafo LangGraph que acota
> tools por estado); Fase 4a · H4.2 (flujo único LangGraph, `F4a.5`); Fase
> 4b · `F4b.3` (solo ahí se decide si H4.2 y H2.3 se separan en dos
> mecanismos). Estado: sin empezar.

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

> **Tesis:** Fase 4a · `F4a.6` (validación del pipeline en el CR5 físico,
> con los límites de seguridad reforzados antes de ejecutar nada emitido
> por el LLM); Fase 5 (la tarea "documentar qué demostró el prototipo" es
> el capítulo de discusión de la tesis). Estado: parcial.

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

> **Tesis:** fuera del camino crítico (decisión de A.1: plataforma propia
> sobre el CR5). No bloquea ninguna fase; se retoma si aparece un segundo
> robot o si `gafro` carga URDF genérico "gratis".

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
- [x] `robot_node/node.py::_build_adapter` y `controller_node/node.py::_build_adapter`
      (04/09): registro `str -> factoría` en vez de if/elif — añadir un
      robot/estrategia nueva es añadir una entrada al dict, no una rama.
      Límite explícito del patrón, sin resolver: `Cr5RealRobotAdapter`
      sigue con `host="192.168.1.100", port=29999` hardcodeado, porque el
      registro no inventa de dónde saldrían esos datos de conexión para un
      segundo robot real — eso sigue pendiente en Bloque 0 (#113).
- [ ] `commander_node.py::main()` (demo) y su comentario sobre que
      `cr5_base.ttt` "no trae un dummy tip dedicado" — una vez exista
      carga de escena general, esa clase de suposición (qué objeto sirve
      de tip si la escena no lo declara explícitamente) tiene que
      resolverse por convención documentada o por manifest, no caso a
      caso como ahora.

## Bloque 10 — Dinámica de la cadena cinemática

> **Tesis:** Fase 4b · `F4b.1`, solo si el MPC conforme de GAFRO necesita
> un modelo dinámico del CR5 (masas e inercias del URDF). Hasta entonces es
> objetivo de la beca, no de la tesis.

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

> **Tesis:** fuera del alcance de la tesis, pendiente de confirmar con el
> director/a qué parte de la beca se cubre con ella (decisión prevista para
> el Año 1 T2 de la hoja de ruta).

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

> **Tesis:** fuera del núcleo H1–H4. El requisito de seguridad mínimo
> (persona como obstáculo del Bloque 4) sale casi gratis con la percepción
> de la Fase 4a, pero no es objetivo de la tesis.

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
