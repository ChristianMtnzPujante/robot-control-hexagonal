"""Protocolo TCP/IP real del CR5 (driver Dobot) -- vía "reimplementar",
la elegida en la decisión de Vikunja Bloque 0 #22 sobre la alternativa de
montar un puente ros1_bridge.

CORREGIDO 04/09 contra el manual OFICIAL del fabricante -- la primera
versión de este módulo se basó solo en leer el driver de referencia
(ROS1/catkin, `~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_bringup`, fechado
2021/08/09) y asumía un puerto de movimiento separado (30003) con un
comando `JointMovJ(j1,...,j6)` de argumentos posicionales. Ese driver
resultó documentar una versión ANTERIOR del protocolo: el manual oficial
`~/ros2_ws/src/TCP-IP-ROS-6AXis/misc/Dobot TCP_IP二次开发接口文档_V4.6.5_20251015_cn.pdf`
(2025/10/15, muy posterior al driver) confirma que el CR5 solo abre
29999/30004/30005/30006 -- el 30003 NO existe -- y que el movimiento
articular se manda como `MovJ(joint={j1,...,j6})` por el MISMO puerto
29999 (sección 2.7, "Parámetros"/"MovJ"). Moraleja concreta: un driver de
referencia enseña qué protocolo hablaba ALGUIEN, no necesariamente el que
habla el firmware actual -- el manual versionado del fabricante es la
fuente de verdad, y aun así habría que confirmar la versión contra el
propio robot antes de fiarse a ciegas (ver checklist de Bloque 0, #112).

Lo que SÍ se mantuvo del driver, porque el manual oficial lo confirma
byte a byte (sección 3, "实时反馈信息"): el formato de la trama real-time
de 1440 bytes, con `TestValue` (valor mágico `0x0123456789ABCDEF`) en los
bytes 48-55 y `QActual` (posición articular real, 6 doubles) en los bytes
432-479 -- esta parte del protocolo no cambió entre 2021 y 2025.

Puertos (constantes del fabricante, no configuración de este repo):
  - 29999 "Dashboard": TODOS los comandos -- de estado (`EnableRobot()`,
    `ClearError()`...) Y de movimiento (`MovJ(joint={...})`). Sin
    `EnableRobot()` previo, el controlador rechaza cualquier movimiento.
    Y antes incluso de `EnableRobot()`: el manual exige `RequestControl()`
    como primer comando de toda la sesión ("solo en modo TCP se pueden
    ejecutar otros comandos TCP", sección 2.1) -- solo se admite si el
    robot está sin energizar o des-energizado (falla si ya está habilitado
    desde el teach pendant). Descubierto en la primera prueba real contra
    el robot físico (ver Cr5RealRobotAdapter._ensure_enabled): sin este
    paso, el primer intento de EnableRobot() se encontró con la conexión
    reseteada por el robot en vez de un código de error normal.
  - 30004 "real-time": no es un socket de comandos, es un STREAM continuo
    de tramas binarias de tamaño fijo con el estado completo del robot.
    (30005/30006 son variantes de la misma información a menor frecuencia
    -- este repo no las usa.)

Formato de comando/respuesta: cada comando es una cadena ASCII sin salto
de línea ("EnableRobot()", "MovJ(joint={10.0,...})"); la respuesta llega
igual ("0,{},EnableRobot();") y basta el primer entero (antes de la
primera coma) como código de error -- 0 es éxito (manual oficial, sección
"消息格式" / "Formato de mensaje").

Formato de la trama real-time: 1440 bytes fijos. Por sí solo un `recv()`
no garantiza que el corte caiga justo en el borde de una trama, así que
hay que resincronizar buscando el valor mágico. Aquí solo necesitamos el
dato MÁS RECIENTE (no un histórico), así que `_extract_last_frame` busca
la última trama válida del buffer en vez de llevar un buffer circular
como el driver de referencia.

SIN VERIFICAR TODAVÍA CONTRA EL ROBOT FÍSICO -- no disponible en este
entorno. Cubierto por tests contra un servidor TCP de mentira que imita
este mismo protocolo (ver src/robot_node/test/test_cr5_protocol.py), pero
eso valida "hablamos el protocolo que documenta el manual", no "el CR5
real responde así". Antes del primer uso real, completar el checklist
físico/de red de Bloque 0 (#112).
"""

from __future__ import annotations

import socket
import struct
from typing import List, Optional, Tuple

from shared_kernel import RobotConnectorError

DASHBOARD_PORT = 29999
REALTIME_PORT = 30004

# RobotMode() (manual, sección de consulta -- NO la de control, así que no
# requiere haber pedido antes el modo TCP con RequestControl(); útil
# precisamente para decidir si RequestControl() va a funcionar, ver
# Cr5RealRobotAdapter.describe_robot_mode).
ROBOT_MODE_DESCRIPTIONS = {
    1: "inicializando",
    2: "con el freno de algún joint suelto (modo manual/mantenimiento)",
    3: "apagado, sin energizar",
    4: "des-energizado y sin freno suelto -- lo que RequestControl() necesita",
    5: "habilitado e inactivo (probablemente encendido desde el panel/teach pendant)",
    6: "en modo de arrastre manual (drag)",
    7: "en ejecución (movimiento o programa en curso)",
    8: "en un movimiento puntual (jog / RunTo)",
    9: "con una alarma sin limpiar (ClearError la limpia si la causa ya no está)",
}
# Los dos únicos estados en los que el manual permite RequestControl():
# "未上电" (sin energizar, código 3) y "下使能, sin freno suelto" (código 4).
CONTROLLABLE_ROBOT_MODES = {3, 4}

_REALTIME_FRAME_LENGTH = 1440
_TEST_VALUE_OFFSET = 48
_EXPECTED_TEST_VALUE = 0x0123456789ABCDEF
# q_actual: 6 doubles (grados), bytes 432-479 de RealTimeData -- posición
# articular actual, exactamente lo que RobotConnectorPort.get_current_configuration
# necesita.
_Q_ACTUAL_OFFSET = 432
_Q_ACTUAL_FORMAT = "<6d"


class Cr5ProtocolError(RobotConnectorError):
    """Fallo de comunicación o de comando contra el CR5 real: timeout,
    desconexión, respuesta irreconocible, o el propio robot devolviendo un
    código de error distinto de 0.

    Hereda de RobotConnectorError (shared_kernel) a propósito: robot_node
    captura ese tipo genérico, no este -- así puede seguir sin saber que
    el CR5 (ni su protocolo) existe (ver Bloque 0 #116)."""


class Cr5CommandSocket:
    """El socket de comandos (puerto 29999, "Dashboard" -- de estado Y de
    movimiento, ver docstring del módulo): manda una cadena ASCII, espera
    la respuesta delimitada por ';' y devuelve su código de error.

    CORREGIDO 07/09, tras la primera sesión real completa (Vikunja Bloque
    0 #110/#116): se conecta sola (si hace falta) y reconecta-y-reintenta
    UNA vez si el envío falla por un problema de conexión (reset, EOF,
    timeout) -- no antes: la primera versión no reconectaba nunca, así que
    un solo hipo de red (visto en vivo más de una vez este bloque: idle
    de varios segundos, latencia de la primera respuesta, reset a mitad
    de una ráfaga de comandos) bastaba para tumbar a quien la usara
    (robot_node moría entero, ver #116).

    Por qué reintentar es seguro aquí y no es la "magia" que se quería
    evitar: TODOS los comandos de este protocolo son idempotentes o de
    solo lectura -- RequestControl/EnableRobot/DisableRobot no hacen daño
    si se piden dos veces, RobotMode() no cambia nada, y MovJ manda una
    posición ABSOLUTA (no un delta), así que reenviarlo de más no acumula
    movimiento, como mucho repite el mismo destino. Un solo reintento
    (no un bucle) sigue evitando la reconexión automática indefinida ante
    un robot de verdad caído, que es el escenario que sí hay que notar en
    vez de ocultar.
    """

    def __init__(self, host: str, port: int, timeout: float = 6.0):
        # 6.0s, no 2.0s: verificado en vivo (07/09) que la primera consulta
        # tras un rato sin ninguna conexión previa (robot recién comprobado,
        # nadie hablándole por TCP) puede tardar varios segundos en
        # responder -- 3.0s de margen dio timeout, 6.0s bastó. No es el
        # mismo problema que la conexión que muere por INACTIVIDAD (ver
        # docstring de Cr5RealtimeSocket.read_joint_angles_deg): aquello
        # era una conexión ya abierta que dejaba de responder; esto es la
        # latencia de la primera respuesta de una conexión recién abierta.
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None

    def connect(self) -> None:
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._timeout
            )
        except OSError as error:
            raise Cr5ProtocolError(
                f"no se pudo conectar a {self._host}:{self._port} -- {error}"
            ) from error

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def send_command(self, command: str) -> int:
        """Manda `command` (p. ej. "EnableRobot()") y devuelve el código de
        error de la respuesta (0 = éxito)."""
        return self.query(command)[0]

    def query(self, command: str) -> Tuple[int, str]:
        """Como send_command, pero además devuelve el contenido entre
        llaves de la respuesta tal cual (sin trocear por comas) -- para
        comandos de consulta como RobotMode() cuyo dato interesante no es
        el ErrorID sino el propio valor devuelto.

        Conecta sola si hace falta, y si el envío falla por un problema de
        conexión reconecta y reintenta UNA vez (ver docstring de la clase)
        antes de rendirse."""
        if self._sock is None:
            self.connect()
        try:
            return self._send_and_parse(command)
        except Cr5ProtocolError:
            self.close()
            self.connect()
            return self._send_and_parse(command)

    def _send_and_parse(self, command: str) -> Tuple[int, str]:
        try:
            self._sock.sendall(command.encode("ascii"))
            response = self._recv_until(b";")
        except OSError as error:
            raise Cr5ProtocolError(
                f'fallo enviando "{command}" a {self._host}:{self._port} -- {error}'
            ) from error
        return _parse_response(response)

    def _recv_until(self, delimiter: bytes) -> bytes:
        buffer = b""
        while delimiter not in buffer:
            chunk = self._sock.recv(1024)
            if not chunk:
                raise Cr5ProtocolError(
                    f"conexión cerrada por el robot (puerto {self._port})"
                )
            buffer += chunk
        return buffer


def _parse_error_code(response: bytes) -> int:
    # Mismo criterio que el driver oficial (parseString en commander.h):
    # el primer entero de la respuesta, antes de la primera coma, es el
    # código de error -- "0,{},EnableRobot();" -> 0.
    text = response.decode("ascii", errors="replace")
    head = text.split(",", 1)[0].strip()
    try:
        return int(head)
    except ValueError as error:
        raise Cr5ProtocolError(f'respuesta no reconocida del CR5: "{text}"') from error


def _parse_response(response: bytes) -> Tuple[int, str]:
    # "ErrorID,{value,...},Comando(...);" -> (ErrorID, "value,..."), tal
    # cual entre llaves -- cada llamante decide cómo interpretar el
    # contenido (RobotMode() solo lo necesita como un único entero).
    error_code = _parse_error_code(response)
    text = response.decode("ascii", errors="replace")
    start = text.find("{")
    end = text.find("}", start + 1) if start != -1 else -1
    if start == -1 or end == -1:
        raise Cr5ProtocolError(f'respuesta no reconocida del CR5: "{text}"')
    return error_code, text[start + 1 : end]


class Cr5RealtimeSocket:
    """El socket de feedback (puerto 30004): un stream continuo de tramas
    binarias de tamaño fijo, no de comandos ASCII -- ver el docstring del
    módulo para el porqué de `_extract_last_frame`."""

    def __init__(self, host: str, port: int = REALTIME_PORT, timeout: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None

    def connect(self) -> None:
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._timeout
            )
        except OSError as error:
            raise Cr5ProtocolError(
                f"no se pudo conectar a {self._host}:{self._port} -- {error}"
            ) from error

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def read_joint_angles_deg(self) -> List[float]:
        """Devuelve q_actual (grados) de la trama MÁS RECIENTE.

        "Más reciente" es la parte delicada: el CR5 transmite continuamente
        (varias tramas por segundo) tanto si alguien lee como si no, así
        que el buffer del sistema operativo puede acumular más de una
        trama entre una llamada y la siguiente. Leer una sola vez y
        quedarnos con lo primero que llegue devolvería la trama más
        ANTIGUA del backlog, no la posición actual del robot -- justo lo
        contrario de lo que RobotConnectorPort.get_current_configuration
        promete. `_drain` vacía el backlog entero antes de extraer.
        """
        if self._sock is None:
            raise Cr5ProtocolError("socket real-time no conectado")
        buffer = self._drain(min_bytes=_REALTIME_FRAME_LENGTH)
        try:
            frame = _extract_last_frame(buffer)
        except Cr5ProtocolError as error:
            raise Cr5ProtocolError(
                "no llegó ninguna trama real-time reconocible"
            ) from error
        return list(struct.unpack_from(_Q_ACTUAL_FORMAT, frame, _Q_ACTUAL_OFFSET))

    def _drain(self, min_bytes: int) -> bytes:
        # Cota defensiva: si el stream nunca contuviera el valor mágico
        # (dato corrupto/no es realmente el CR5), no acumular memoria sin
        # límite -- nos quedamos solo con lo último leído.
        max_buffer = _REALTIME_FRAME_LENGTH * 8
        buffer = b""
        try:
            # Fase 1, bloqueante (con el timeout normal del socket): hasta
            # tener al menos una trama de margen. Sin esto, un backlog
            # vacío en el momento exacto de la fase 2 haría devolver
            # buffer="" en vez de esperar a que llegue algo.
            while len(buffer) < min_bytes:
                chunk = self._sock.recv(_REALTIME_FRAME_LENGTH * 4)
                if not chunk:
                    raise Cr5ProtocolError("conexión real-time cerrada por el robot")
                buffer += chunk
            # Fase 2, no bloqueante: vacía cualquier trama más nueva que ya
            # estuviera esperando en el buffer del sistema operativo desde
            # antes de esta llamada.
            self._sock.settimeout(0)
            try:
                while True:
                    try:
                        chunk = self._sock.recv(_REALTIME_FRAME_LENGTH * 4)
                    except (BlockingIOError, socket.timeout):
                        break
                    if not chunk:
                        break
                    buffer += chunk
                    if len(buffer) > max_buffer:
                        buffer = buffer[-max_buffer:]
            finally:
                self._sock.settimeout(self._timeout)
        except OSError as error:
            raise Cr5ProtocolError(
                f"fallo leyendo el stream real-time de {self._host}:{self._port} -- {error}"
            ) from error
        return buffer


def _extract_last_frame(buffer: bytes) -> bytes:
    for start in range(len(buffer) - _REALTIME_FRAME_LENGTH, -1, -1):
        end = start + _REALTIME_FRAME_LENGTH
        candidate = buffer[start:end]
        (test_value,) = struct.unpack_from("<Q", candidate, _TEST_VALUE_OFFSET)
        if test_value == _EXPECTED_TEST_VALUE:
            return candidate
    raise Cr5ProtocolError(
        "no se encontró ninguna trama real-time válida (valor mágico ausente)"
    )
