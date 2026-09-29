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

- [ ] **Efector final: el CR5 tiene una pinza montada por el conector de 8
      pines de la brida, pero NO responde a ningún sondeo (17/09).** El
      repo no tiene hoy nada de pinza (ni puerto, ni método en
      `Cr5RealRobotAdapter`, ni comandos `ToolDO`/Modbus), y antes de
      diseñar ese soporte hace falta saber POR DÓNDE se manda. Lo que se
      sabe y lo que no, medido contra el robot físico:

      El conector es el "End I/O" del manual de usuario del CR5 (Tabla 3.5,
      cable Lumberg RKMV 8-354): pin 1 `AI_1/485A`, 2 `AI_2/485B`, 3 `DI_2`,
      4 `DI_1`, 5 `24V (Out)`, 6 `DO_2`, 7 `DO_1`, 8 `GND`. O sea
      alimentación + 2 DI + 2 DO + RS485 multiplexado con las analógicas,
      y el modo por defecto del terminal multiplexado es **485** (manual
      TCP/IP V4.6.5, `SetToolMode`) — por eso `ToolAI` devuelve 0 y no
      significa nada mientras no se conmute a modo analógico.

      Sondeo con el robot ENERGIZADO (modo 5, sin alarmas) y tras ciclar la
      alimentación del extremo (`SetToolPower(0)` → `SetToolPower(1)`, que
      el manual describe justo como "re-inicializar la pinza"):
      `ToolDI(1)`=`ToolDI(2)`=0, `GetToolDO(1)`=`GetToolDO(2)`=0, y
      **ningún esclavo Modbus RTU contesta**: `ModbusRTUCreate` crea el
      maestro sin problema (eso solo abre el puerto en el lado del robot,
      no prueba que haya esclavo) pero `GetHoldRegs` devuelve -1 en los 15
      slave id (1-15) a 115200 8N1, y también en 9600/19200/38400/57600 y
      en 115200 8E1 para los id 1 y 9. Tampoco hay nada en las E/S de la
      caja de control (`digital_input_bits`/`digital_outputs` = 0).

      Queda por descartar, en este orden: (a) que sea una pinza de control
      DIGITAL por `ToolDO` — la prueba definitiva es conmutar `ToolDO`, que
      MUEVE los dedos y por tanto exige a alguien delante; (b) parámetros
      de 485 fuera de lo probado (7 bits, paridad impar, 230400/921600,
      slave id >15); (c) que el cable de 8 pines solo le lleve corriente y
      el control vaya por otro sitio (controlador propio de la pinza
      cableado a la caja, no al brazo). Lo que desbloquea todo esto es
      saber **marca y modelo**, que decide slave id, baudios y mapa de
      registros.

      **CORRECCIÓN (17/09, mismo día): el sondeo no valía para esta
      pinza.** El usuario identificó el modelo: **Robotiq 2F Adaptive
      Gripper** (2F-85/2F-140). Sus parámetros de fábrica son slave id
      **9**, 115200 8N1 — que sí entraban en el barrido — pero su mapa de
      registros NO: el estado se lee con FC03 en **0x07D0** (2000) y la
      petición de acción se escribe en **0x03E8** (1000). El barrido
      limpio solo leyó 0x0200 y 0x0000 para el id 9, direcciones que esta
      pinza no implementa, y una dirección inexistente devuelve excepción
      Modbus —que `GetHoldRegs` reporta como -1, idéntico a "nadie
      contesta"—. O sea que **el resultado negativo no distingue "no hay
      pinza en el 485" de "pregunté por la dirección equivocada"**. La
      única pasada que sí probó 0x07D0 para el id 9 fue la primera, la que
      iba desincronizada y hubo que descartar por poco fiable.

      Prueba pendiente, con el robot encendido:
      `ModbusRTUCreate(9,115200,"N",8,1)` y `GetHoldRegs(idx,2000,3)` —
      debería devolver los registros de estado (gACT/gGTO/gSTA/gOBJ). Si
      sigue sin contestar con la dirección correcta, el siguiente
      sospechoso es el cableado del conector de 8 pines: **485A/485B
      cruzados** es el fallo típico de un cable a medida, y encaja con
      "alimenta pero nunca responde". Ojo también a que el 2F hay que
      ACTIVARLO (escritura en 0x03E8) antes de que mueva, aunque para
      responder a una lectura de estado no hace falta.

      **RESUELTO A NIVEL DE DIAGNÓSTICO (18/09): el problema es el CABLE,
      no el software.** Dato que lo cambia todo: con el extremo alimentado,
      la pinza enciende un **LED rojo FIJO**. Según la tabla de fallos del
      manual Robotiq (§4.4), rojo fijo es fallo MENOR, y de los dos
      posibles (`0x08` exceso de temperatura, `0x09` "sin comunicación
      durante al menos 1 segundo") solo encaja el segundo. Y es
      concluyente por eliminación: si la pinza comunicara pero estuviera
      sin activar, el LED sería AZUL (`0x07` es fallo de prioridad, LED
      azul). O sea: **está viva, alimentada, y NO le llegan los datos**.

      Descartado ya TODO lo que se puede descartar por software, en esta
      orden: dirección correcta (2000, FC03); slave id 9 y barrido
      completo 1-16; 115200 8N1 y también 8E1; `SetToolPower(1)`
      confirmado con `0,{}` (y ciclado off/on); robot energizado (modo 5)
      además de des-energizado; y `SetToolMode(1)` + `SetTool485` para
      forzar el terminal multiplexado a 485 por si alguien lo había
      dejado en analógico. Nada contesta en ningún caso.

      Queda UN sospechoso, tras leer las figuras 3-9/3-5.1 del manual
      Robotiq (que pdftotext no renderiza -- hay que abrir el PDF):
      **el par de datos cruzado**. El cable de dispositivo de Robotiq es
      de 8 polos pero solo usa 4 señales: pin 1 `24V`, pin 2 `GND`, pin 3
      `RS485+`, pin 4 `RS485-`. El mapeo a la brida del CR5 debería ser
      1->5 (24V), 2->8 (GND), y 3/4 -> 1/2 (485A/485B). **Que la pinza
      encienda demuestra que 1 y 2 están bien**; lo único sin verificar es
      3/4, y ahí la correspondencia `485+`/`485-` <-> `485A`/`485B` NO es
      estándar (en TIA/EIA-485 la "A" es la invertida, pero medio sector
      la etiqueta al revés). Además el montaje del laboratorio va con un
      cable INTERMEDIO, y la propia figura 3-9 dibuja el extremo macho y
      el hembra con los pines 3 y 4 en posiciones espejadas -- el
      fabricante avisa del error precisamente porque es el habitual.
      Un espejado COMPLETO está descartado: pondría los 24V en `DI_1` y la
      pinza no encendería.

      DESCARTADO (corregido el 18/09): la hipótesis de "`485 GND` sin
      conectar" no aplica. El acoplamiento tiene ese pin (el 10 de su
      bornera de 10), pero **el cable de dispositivo de Robotiq solo lleva
      4 conductores y no lo incluye**, así que su ausencia es lo normal.

      DESCARTADO (24/09): el orden de configuración. Sospecha: como
      `SetToolPower` resetea la conexión del 29999, quizá reinicia la
      placa del extremo y deshace un `SetToolMode`/`SetTool485` previo.
      Con alimentación PRIMERO y luego modo 485 + formato + maestro,
      `GetHoldRegs(0,2000,3)` sigue en `-1`, cada uno a 0,50 s exactos
      (el controlador espera respuesta, no rechaza al instante), sin
      alarmas. `docs/cr5_485_scope_test.py` queda con el orden nuevo.
      Sin espera entre `SetToolPower` y el resto (0,01 s toda la
      configuración): igual, `-1`. Ojo: la herramienta ya estaba
      alimentada, así que no cubre un arranque en frío.
      Otros códigos de función, mismo resultado (`-1`, 0,50 s todos):
      FC04 `GetInRegs`, FC01 `GetCoils` y FC02 `GetInBits` (no
      soportadas por la pinza: habrían provocado una excepción Modbus si
      la trama llegara) y FC16 `SetHoldRegs(1000,{0,0,0})` (rACT=0, no
      mueve). El fallo no depende del tipo de petición.
      Barrido de velocidad con el orden nuevo: 2400-230400 bps × N/E/O,
      slave 9: `-1` en todas, 0,50 s fijos (el timeout no depende de la
      velocidad). Repite el negativo del 18/09 y añade 2400/4800.
      No hay `GetToolMode` ni campo en el 30004: el modo del terminal no
      se puede leer, solo fijar.
      Aparte: el adaptador USB-Ethernet (ASIX AX88179) entró en bucle de
      desconexión cada 3-4 s (~240 en 30 min) — mismo fallo que el 21/09,
      peor. Sospechoso de puerto/cable USB.
      Cortocircuito accidental al pinchar la brida con el osciloscopio
      (24/09): alarma **1487** (no está en las tablas de `rosdemo_v4`),
      modo 9. Se recupera con `ClearError()` -> modo 4, y `SetToolPower(1)`
      vuelve a dar `0` sin alarma. La medida anterior a eso no vale: las
      sondas estaban en los pines 7/3, no en el 1/2.

      **RESUELTO (29/09): el problema era el COMANDO, no el cable.**
      `ModbusRTUCreate` no llega al 485 de la brida. A la brida se llega con
      `ModbusCreate("127.0.0.1",60000,9,1)` (paso directo del controlador
      al extremo), tras `SetToolPower(1)`, `SetToolMode(1)` y
      `SetTool485(115200,"N",1)`. La pinza contestó a la primera, en 0,02 s:
      `{256,2304,768}` → gFLT=0x09 (el LED rojo fijo de siempre), gPO=3.
      Reset + activación (`SetHoldRegs(...,{0,0,0})` → `{256,0,0}`) dejan
      gSTA=3 y gFLT=0x00. Orden de bytes confirmado: byte alto primero.
      Fuente: ejemplo oficial Dobot+ "Control End Gripper"
      (`examples/Basic/grip`, una Robotiq EPick) y la doc V3 del protocolo
      ("60000 terminal transparent port"); el manual V4.6.5 no lo dice.
      Adaptador y `docs/cr5_485_scope_test.py` cambiados a esa vía.
      Abrir/cerrar verificado el mismo día (vel/fuerza 64): cierra y abre
      en 1,8 s, topes reales `gPO` 3 (abierta) y 228 (cerrada sin objeto),
      corriente en vacío ≤ 110 mA. Con gFLT=0x09 activo (2 s sin hablarle)
      la pinza ACEPTA la orden y el fallo se borra: no hace falta sondeo
      continuo para mandar órdenes. Pendiente: confirmar si
      `ModbusRTUCreate` sale por el 485 del armario
      (`docs/cr5_controller485_probe.py`); `opening` = gPO/255 da ~0,89 con
      la pinza cerrada del todo (topes 3-228).
      Ajustes tras leer el SDK oficial de Robotiq (`robotiq/grippers`):
      `activate()` ya NO reactiva una pinza activada y sin fallo grave
      (reactivar la abre y cierra entera y suelta lo agarrado), y
      `fault_code` toma solo los bits 0-3 del byte 2 (los 4-7 son kFLT).
      Chuleta de uso en el vault: "Pinza Robotiq 2F - Uso práctico".
      **Primera secuencia brazo + pinza (29/09):**
      `commander/lift_and_grip_demo.py` sube el TCP 5 cm (PoE, 21
      waypoints), espera a RobotMode()==5 y abre/cierra la pinza por el
      mismo socket, con `Cr5RealRobotAdapter` + `Robotiq2FGripperAdapter`.
      Contra el robot real: subida medida +49,9 mm, desvío XY 0,0 mm; la
      pinza abrió (0,01) y cerró (0,90) sin fallo. Es la primera prueba
      del ADAPTADOR de la pinza (no de un script suelto) contra hardware;
      falta el camino ROS (topics de robot_node).
      **Camino ROS verificado (29/09):** `robot_node` real con
      `gripper_target:=robotiq_2f`, órdenes 0.0 → 1.0 → 0.5 por
      `/gripper_command`, leído 0,502. Sin pinza configurada, solo avisos.
      El adaptador ahora marca la pinza como no disponible tras un `-1`
      (no vuelve a ocupar el socket del brazo hasta `close()`), y
      `robot_node/package.xml` declara `std_msgs`. Pendiente:
      `gripper_state`; el `RCLError` al parar con Ctrl+C es de
      `ros2_kit/runner.py` (doble `rclpy.shutdown()`), no de la pinza.

- [ ] **`load` = 0 kg con una herramienta calibrada (17/09).** La trama
      real-time dice `toolCoordinate`=1 con un TCP de
      (-18.08, -45.23, 152.85) mm — alguien midió el efector y lo guardó —
      pero `load` y el centro de masa están a 0. El controlador cree que va
      descargado: afecta a la compensación de gravedad y a los umbrales de
      detección de colisión (los mismos que dispararon la alarma [76] del
      semicírculo). Si la pinza pesa algo apreciable, falta configurar
      `PayLoad(peso, excentricidad)`.

- [ ] **Hallazgo de protocolo (17/09): el puerto 29999 admite UN SOLO
      cliente.** Reconectando justo después de cerrar, el controlador
      acepta el TCP y contesta literalmente
      `Connection refused, IP:Port has been occupied` durante unos
      segundos, en vez de rechazar la conexión o devolver un código de
      error del protocolo. `Cr5CommandSocket.connect` no contempla ese
      caso: hoy daría por buena la conexión y ese texto se colaría como
      respuesta del primer comando. Además, una ráfaga de comandos que
      fallan (`GetHoldRegs` contra un esclavo inexistente) acaba con el
      controlador **reseteando la conexión** — mismo síntoma ya
      documentado en `_cr5_protocol.py` para comandos no admitidos en el
      estado actual, no un error limpio.

- [ ] **`Cr5RealtimeSocket` no reconecta nunca, y el ciclo de energización
      le mata el stream (17/09, visto en vivo dos veces).** El socket de
      30004 se abre en el `__init__` de `Cr5RealRobotAdapter` y
      `read_joint_angles_deg` no tiene ningún reintento, a diferencia de
      `Cr5CommandSocket.query`, que sí reconecta sola. Si entre la
      construcción del adaptador y la primera lectura pasa un
      `DisableRobot()`/`EnableRobot()` —lo normal cuando el robot llega ya
      habilitado y hay que des-energizarlo para que `RequestControl()` sea
      admisible— el stream se muere y `get_current_configuration` falla con
      "timed out" para SIEMPRE en esa instancia. Pasó bajando el TCP a
      40 cm: el adaptador mandó el primer `MovJ` (el robot SE MOVIÓ) y
      luego no pudo leer dónde había quedado. Y no es un caso de borde:
      con el reintento puesto a mano en el script, el stream volvió a
      caerse a mitad del recorrido (tramo 15 de 18) y solo siguió porque
      había reconexión. Arreglo natural: que `read_joint_angles_deg`
      reconecte igual que `query`, y/o que la conexión de 30004 sea
      perezosa en vez de hacerse en el `__init__`.

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
- [x] **(29/09)** `cr5_poe_vs_gafro_simple_demo.py` explica el cálculo paso
      a paso con números reales (FK, error inicial, Jacobiano, tabla de
      iteraciones, resultado; `--no-sim` para verlo sin CoppeliaSim), con
      una prueba 3 nueva que cambia también la orientación. Los dos
      adaptadores guardan `last_trace` (una `IkIteration` por iteración,
      solo diagnóstico). *Hallazgo:* el Jacobiano es idéntico (4e-16 tras
      reordenar la base) y la parte rotacional del error también; la
      diferencia está en la traslacional. El `log()` del motor de gafro
      devuelve la traslación del motor de error tal cual, mientras que el
      twist de PoE es el tornillo v = G(θ)⁻¹·p. Con traslación pura son
      idénticos; con 46,5° de giro la traslación difiere 0,17 y GA aleja
      la punta en la 1ª iteración (100 → 197 mm) y tarda 6 iteraciones
      frente a 4 (misma solución, 0,11 mrad). Y matiz a lo del 17/09: la
      diferencia articular en la home (2°) NO viene del álgebra: las
      cuentas son iguales hasta la iteración 2 y el redondeo se amplifica
      al salir de la singularidad (Jacobiano de rango 3).
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
- [ ] **(23/09) Decisión de diseño abierta: tareas CGA por primitivas y
      separación planificador/ejecutor.** Surgió al preguntar hasta dónde
      llega `GaKinematicsAdapter` (solo la IK de pose a pose, igual que PoE)
      y dónde encajarían tareas como "sigue esta línea". Resultado de la
      discusión, sin código de producción todavía:
      - `KinematicsPort` **no se extiende**: se queda como IK de pose a pose,
        el denominador común de PoE/GA/DH/simulador. Añadirle `follow_line`
        obligaría a PoE a lanzar `NotImplementedError` y a los consumidores
        a comprobar tipos con `isinstance`.
      - **Puerto nuevo de tareas** (nombre provisional `GeometricTaskPort`):
        para una restricción geométrica y una configuración θ devuelve el
        par **(residuo e(θ), Jacobiano J(θ))**, no una trayectoria resuelta.
        Lo implementaría GA (la misma clase puede cumplir los dos puertos).
        Una tarea es "la primitiva X del robot (punto, recta del eje de la
        herramienta) es incidente con la primitiva A": X ∧ A = 0. Aparte
        quedan pocas especiales, como el paralelismo (sobre direcciones:
        dir(L) ∧ dir(L_goal) = 0).
      - **Combinar restricciones**: las restricciones duras sobre el MISMO
        punto se fusionan geométricamente con el meet, A∩B = (A*∧B*)*. El
        grado del resultado cuenta las ecuaciones independientes, un 0 indica
        redundancia y un resultado en el infinito indica conflicto
        (verificado en pygafro). El resto se combina en la capa numérica:
        apilado, o prioridades con proyector al espacio nulo. El meet no
        expresa pesos ni prioridades, y las desigualdades (evitar un
        obstáculo) no son incidencias: van como tareas activadas por
        proximidad, con máxima prioridad, o en un QP/MPC. Para diagnosticar
        sí sirven los signos de X·S* (dentro/fuera) y del cuadrado del par
        de puntos L∩S (atraviesa/tangente/no toca).
      - **Separar planificador y ejecutor.** El planificador (global, Bloque
        4) emite tramos de primitivas CGA combinadas con su condición de fin;
        los puntos de transición son intersecciones (L₁∩L₂, L∩Π). El
        ejecutor (local) los resuelve con residuo + Jacobiano y prioridades.
        Al principio, integrando el tramo antes de enviarlo al robot y
        produciendo una `Trajectory` en articulaciones, así `robot_node` no
        cambia. Más adelante, en línea (MPC, Fase 4b). Encaja con F1.4
        (tools con esquema CGA) y F1.6 (verificar antes de actuar).
      - **Cuestiones abiertas:** (a) ¿tipos neutros de `geometry_kernel` o
        tipos conformes en el puerto? (la opción neutra matiza la decisión de
        bounded contexts de arriba); (b) el planificador necesita que el
        ejecutor le confirme si un tramo es ejecutable para el brazo entero
        (colisiones, límites, singularidades): bucle proponer → verificar →
        replanificar; (c) el tramo debe poder fijar rama/configuración
        preferida o una tarea secundaria (lección del 14/09); (d) paralelo
        frente a antiparalelo: `Bᵀu = 0` acepta los dos, hay que usar `u − u_g`
        si importa el sentido.
      - **Evidencia numérica** (CR5, pygafro): `docs/cga_tareas_linea_prioridades.py`
        (herramienta coaxial con una recta más avance de 10 cm: apilado
        converge en 6 iteraciones; con prioridades, la tarea 1 se mantiene
        en ~1e-5 mientras avanza; tras la prioridad 1 quedan 2 GDL libres;
        los Jacobianos coinciden con diferencias finitas a ~4e-8) y
        `docs/cga_meet_vs_apilado.py` (plano + esfera apilados frente a su
        meet, el círculo: mismo conjunto válido, soluciones distintas en
        ~1e-4 rad).
      - **Siguiente paso** (cuando toque): definir las firmas a partir de un
        primer consumidor real, una demo de "sigue esta línea" en
        CoppeliaSim, no antes.
- [ ] **(29/09) OBJETIVO INICIAL — artículo de revisión + propuesta de
      ontología del dominio.** Fusiona el hito de revisión de la Fase 1 de
      la tesis (F1.3) con la ontología de abajo, y añade una parte
      práctica: escenas de ejemplo definidas con este repo (`Scene`,
      `geometry_kernel`, ficheros de `FilePerceptionAdapter`,
      `build_cr5_scene`), descritas en la ontología, traducidas a CGA y con
      capturas de CoppeliaSim, propuestas como banco para la fase
      siguiente (H1.1). Validación mínima: expresividad (las escenas se
      describen), ejecutabilidad (una relación resuelta de punta a punta),
      y uso por un LLM (opcional). Pendiente: confirmarlo con el director.
      Plan en el vault, `Investigación/Ontología del Dominio (lenguaje CGA).md`.
- [ ] **(29/09) Investigar: ontología del dominio / lenguaje propio.**
      Definir los objetos y relaciones del dominio (robot, escena, objetos,
      tareas) como un lenguaje completo que sirva a solvers clásicos (cada
      término se traduce a residuo + Jacobiano o restricción), que exprese
      sobre todo lo que da CGA (primitivas, incidencia, meet, signos) y que
      sea un vocabulario común para un LLM. Sería el lenguaje de los
      "tramos de primitivas" de la decisión del 23/09 (arriba). Pistas,
      pendientes de leer: Task Frame Formalism / iTaSC (De Schutter,
      Bruyninckx et al.); Kresse y Beetz (KnowRob), restricciones entre
      features punto/línea/plano; De Laet, Bruyninckx et al., "Geometric
      relations between rigid bodies: semantics for standardization" (IEEE
      RAM, 2013). Nota: vault, `Investigación/Ontología del Dominio
      (lenguaje CGA).md`.

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
      **(23/09)** Ver la decisión abierta del Bloque 1 "tareas CGA por
      primitivas y separación planificador/ejecutor": CHOMP/RRT serían el
      planificador global que emite tramos de primitivas CGA, y un
      ejecutor local los resolvería con residuo + Jacobiano. CHOMP puede
      usar directamente esos pares (e, J) como términos de coste.
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
