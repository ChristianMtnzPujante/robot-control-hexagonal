---
tags: [arquitectura, adaptador]
---

# _cr5_protocol (protocolo TCP del CR5)

Módulo de protocolo puro, sin nada de dominio: implementa la conversación
TCP/IP real con el controlador del CR5. Lo consume
[[Cr5RealRobotAdapter]] (que sí conoce `JointConfiguration`/
`RobotConnectorPort`) — esta nota es la referencia función-por-función del
módulo en sí; el porqué de las decisiones de protocolo (puerto 30003
inexistente, `RequestControl()` obligatorio, `cp` de suavizado...) ya está
narrado en [[Cr5RealRobotAdapter]] y en [[Decisiones de Diseño Clave]], no
se repite aquí. Código:
`src/robot_node/robot_node/adapters/_cr5_protocol.py`.

## Constantes y tablas

- `DASHBOARD_PORT = 29999`, `REALTIME_PORT = 30004` — puertos del
  fabricante. `Cr5RealRobotAdapter` los admite como parámetro solo para
  poder apuntar los tests a un servidor TCP de mentira en un puerto
  efímero; en producción son siempre estos.
- `ROBOT_MODE_DESCRIPTIONS` — tabla código → descripción legible de los 9
  estados que devuelve `RobotMode()` (inicializando, sin energizar,
  habilitado e inactivo, en alarma...).
- `CONTROLLABLE_ROBOT_MODES = {3, 4}` — los únicos dos estados ("sin
  energizar" y "des-energizado sin freno suelto") en los que el manual
  admite `RequestControl()`.
- `_REALTIME_FRAME_LENGTH = 1440`, `_TEST_VALUE_OFFSET = 48`,
  `_EXPECTED_TEST_VALUE`, `_Q_ACTUAL_OFFSET = 432`, `_Q_ACTUAL_FORMAT =
  "<6d"` — geometría binaria exacta de la trama real-time.

## `Cr5ProtocolError`

Hereda de `RobotConnectorError` (`shared_kernel`), no es un error propio
del adaptador — así `robot_node` puede capturar el tipo genérico del
puerto sin saber que el CR5 (ni su protocolo) existe.

## `Cr5CommandSocket` — puerto 29999, comandos ASCII

- `__init__(host, port, timeout=6.0)` — 6.0s, no 2.0s: verificado en vivo
  que la primera consulta tras un rato sin ninguna conexión previa puede
  tardar varios segundos en responder.
- `connect()` / `close()` / `is_connected` — gestión básica del socket;
  envuelve `OSError` en `Cr5ProtocolError`.
- `send_command(command) -> int` — manda una cadena ASCII (p. ej.
  `"EnableRobot()"`) y devuelve solo el `ErrorID` de la respuesta.
- `query(command) -> (int, str)` — como `send_command`, pero además
  devuelve el contenido entre llaves de la respuesta sin trocear (para
  comandos de consulta como `RobotMode()`, cuyo dato interesante no es el
  `ErrorID`). Conecta sola si hace falta, y si el envío falla por un
  problema de conexión, **reconecta y reintenta UNA vez** antes de
  rendirse — seguro porque todo el protocolo es idempotente o de solo
  lectura (`MovJ` manda posiciones absolutas, no deltas).
- `_send_and_parse` / `_recv_until(delimiter)` — helpers internos:
  acumulan bytes hasta encontrar el delimitador `;`; lanzan
  `Cr5ProtocolError` si el socket se cierra a media lectura.

## Funciones de parseo de respuesta

- `_parse_error_code(response)` — el primer entero antes de la primera
  coma de la respuesta ASCII (`"0,{},EnableRobot();"` → `0`).
- `_parse_response(response) -> (error_code, contenido_entre_llaves)` —
  mismo criterio que el driver oficial (`parseString` en `commander.h`);
  cada llamante decide cómo interpretar ese contenido.

## `Cr5RealtimeSocket` — puerto 30004, stream binario continuo

- `__init__(host, port=REALTIME_PORT, timeout=2.0)`, `connect()`,
  `close()`.
- `read_joint_angles_deg() -> List[float]` — desempaqueta `q_actual` (6
  doubles, grados) de la trama MÁS RECIENTE disponible.
- `_drain(min_bytes)` — dos fases: (1) bloqueante, hasta acumular al
  menos una trama de margen; (2) no bloqueante, vacía cualquier trama más
  nueva que ya estuviera esperando en el buffer del sistema operativo.
  Cota defensiva de memoria (8 tramas) por si el stream nunca contuviera
  el valor mágico.
- `_extract_last_frame(buffer)` (función de módulo, no método) — recorre
  el buffer de atrás hacia adelante buscando el valor mágico en el offset
  48; lanza `Cr5ProtocolError` si no encuentra ninguna trama reconocible.

**Por qué "la más reciente" importa**: el CR5 transmite continuamente
tanto si alguien lee como si no. Leer una sola vez devolvería la trama
más antigua acumulada en el buffer, no la posición actual del robot —
justo lo contrario de lo que `RobotConnectorPort.get_current_configuration`
promete.

## Ver también

- [[Cr5RealRobotAdapter]]
- [[RobotConnectorPort]]
- [[Decisiones de Diseño Clave]]
