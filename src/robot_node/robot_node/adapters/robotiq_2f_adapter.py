"""Adaptador de la pinza Robotiq 2F Adaptive (GripperPort) a través del
controlador del CR5.

La pinza NO es un dispositivo de red: cuelga del RS-485 de la brida (pines
1 y 2 del conector de 8 pines, ver la guía "E/S del extremo del CR5" del
vault). Quien hace de maestro Modbus es el CONTROLADOR del CR5, así que
aquí no hace falta ni un conversor USB-485 ni pymodbus: se le pide al
controlador que pregunte por nosotros, con los comandos Modbus del puerto
29999 que ya habla _cr5_protocol.py.

POR QUÉ RECIBE EL SOCKET EN VEZ DE ABRIR EL SUYO: el 29999 admite UN SOLO
CLIENTE (comprobado en vivo el 17/09 -- reconectar demasiado pronto
devuelve el texto "Connection refused, IP:Port has been occupied"). Si este
adaptador abriera su propia conexión mientras Cr5RealRobotAdapter tiene la
suya, el controlador la rechazaría. Compartir el socket hace que esa regla
se cumpla POR CONSTRUCCIÓN, y es también la razón de que la pinza viva hoy
en el mismo nodo que el robot y no en uno aparte.

OJO -- el socket es secuencial: mientras viaja un GetHoldRegs no viaja un
MovJ. Un GetHoldRegs contra un esclavo que no contesta tarda ~0.5s, y una
ráfaga de fallos llegó a provocar un reset de la conexión. Por eso este
adaptador NO sondea por su cuenta: solo habla cuando se le llama.

Mapa de registros (manual Robotiq 2F-85/2F-140, §4.2-4.4; PDF en
~/Downloads/manuales_robotiq/):

    escribir en 1000 (0x03E8), 3 registros:
        byte0 = rACT | rGTO<<3 | rATR<<4   byte1 = 0   byte2 = 0
        byte3 = rPR (0 abierta .. 255 cerrada)
        byte4 = rSP (velocidad)   byte5 = rFR (fuerza)
    leer de 2000 (0x07D0), 3 registros:
        byte0 = gACT(bit0) gGTO(bit3) gSTA(bits4-5) gOBJ(bits6-7)
        byte2 = kFLT(bits4-7) gFLT(bits0-3)
        byte3 = gPR   byte4 = gPO   byte5 = gCU

Cada registro son 2 bytes, el par en la parte alta: reg = (b_par<<8)|b_impar.

CÓMO SE LLEGA AL 485 DE LA BRIDA (verificado el 29/09 contra la pinza real):
`ModbusCreate("127.0.0.1",60000,9,1)` -- un maestro RTU por el puerto 60000
del propio controlador, que hace de paso directo al 485 del extremo. NO
`ModbusRTUCreate`: con ese la pinza no contestó nunca (17/09-29/09, siempre
-1 en ~0,5 s); con este respondió a la primera en 0,02 s. Fuente: el ejemplo
oficial de Dobot+ "Control End Gripper" (examples/Basic/grip, control.lua,
una Robotiq EPick) y el "60000 terminal transparent port" de la doc V3 del
protocolo. El manual V4.6.5 no lo dice en ningún sitio.

El formato serie de ese paso lo fija SetTool485, y SetToolMode(1) deja los
pines 1-2 en modo 485 (no analógico): los dos van antes de crear el maestro.
El ejemplo de Dobot+ rodea las lecturas con Use485()/UnLock485(), pero son
funciones de Lua (dentro del controlador), NO comandos del 29999: no están
en el manual V4.6.5 ni en el SDK de Python.

Verificado contra la pinza (29/09, desde script en docs/, con los mismos
comandos que manda este adaptador): lectura del 2000, activación, cerrar y
abrir, y el orden de bytes (byte alto primero). Topes reales de gPO: 3
(abierta) y 228 (cerrada sin objeto). Con gFLT=0x09 activo la pinza acepta
igualmente una orden, así que no hace falta sondeo continuo. Falta probar
el adaptador en sí dentro de robot_node.
"""

from __future__ import annotations

from typing import List, Optional

from shared_kernel import GripperState

from ._cr5_protocol import Cr5CommandSocket, Cr5ProtocolError

# Valores de fábrica de la Robotiq 2F (manual §4.7 "Modbus RTU Communication").
_SLAVE_ID = 9
_BAUD = 115200
_PARITY = "N"
_STOP_BITS = 1

# Paso directo del controlador al 485 de la brida (ver docstring del módulo).
_TOOL485_GATEWAY_IP = "127.0.0.1"
_TOOL485_GATEWAY_PORT = 60000

_ACTION_ADDR = 1000  # 0x03E8, lo que le pedimos
_STATUS_ADDR = 2000  # 0x07D0, lo que está pasando
_REGISTER_COUNT = 3

_DEFAULT_SPEED = 128
_DEFAULT_FORCE = 128

_STA_ACTIVATION_COMPLETED = 3

# gFLT 0x0A-0x0F son fallos GRAVES (LED rojo/azul parpadeando): solo se
# borran con un reset (flanco de subida de rACT). Los de debajo se borran
# solos -- p. ej. 0x09, "sin comunicación 1 s", que la pinza da siempre que
# nadie le habla y que no le impide aceptar órdenes (verificado 29/09).
_FIRST_MAJOR_FAULT = 0x0A


def _pack_action(ract: int, rgto: int, rpr: int, rsp: int, rfr: int) -> List[int]:
    """Los 6 bytes de la petición, empaquetados en 3 U16."""
    byte0 = (ract & 0x01) | ((rgto & 0x01) << 3)
    return [(byte0 << 8) | 0x00, rpr & 0xFF, ((rsp & 0xFF) << 8) | (rfr & 0xFF)]


def _parse_status(registers: List[int]) -> GripperState:
    if len(registers) != _REGISTER_COUNT:
        raise Cr5ProtocolError(
            f"la pinza devolvió {len(registers)} registros, se esperaban "
            f"{_REGISTER_COUNT}: {registers}"
        )
    status = registers[0] >> 8
    return GripperState(
        opening=(registers[2] >> 8) / 255.0,  # gPO
        activated=((status >> 4) & 0x03) == _STA_ACTIVATION_COMPLETED,  # gSTA
        # gOBJ: 1 y 2 son "he parado por contacto" (abriendo/cerrando); 0 es
        # "en movimiento" y 3 "llegué a la posición pedida, no hay nada".
        holding_object=((status >> 6) & 0x03) in (1, 2),
        # El byte 2 lleva kFLT (bits 4-7, controlador opcional de Robotiq)
        # y gFLT (bits 0-3, la pinza): SDK oficial robotiq/grippers.
        fault_code=(registers[1] >> 8) & 0x0F,  # gFLT
    )


class Robotiq2FGripperAdapter:
    """GripperPort sobre el maestro Modbus RTU del controlador del CR5, por el
    paso directo 127.0.0.1:60000 al 485 de la brida."""

    def __init__(self, commands: Cr5CommandSocket):
        self._commands = commands
        self._master_index: int = -1
        self._unavailable_reason: Optional[str] = None

    def _query(self, command: str, what_failed: str) -> str:
        """Todo comando de la pinza pasa por aquí. Si el controlador
        contesta con error (el -1 típico de una pinza ausente o sin
        alimentar), la pinza queda marcada como NO DISPONIBLE y las llamadas
        siguientes fallan al instante, sin tocar el socket.

        Por qué: el socket es el del BRAZO (el 29999 admite un solo
        cliente). Cada GetHoldRegs sin respuesta lo bloquea ~0,5 s, y una
        ráfaga de comandos fallidos llegó a provocar un reset de la
        conexión (17/09). Sin esto, alguien publicando en gripper_command
        con la pinza desconectada podría frenar o cortar el brazo.

        Un fallo de la CONEXIÓN (excepción de query) no marca nada: no es
        culpa de la pinza, y afecta igual al brazo. close() quita la marca
        para poder reintentar."""
        if self._unavailable_reason is not None:
            raise Cr5ProtocolError(
                "pinza marcada como no disponible tras un fallo anterior "
                f"({self._unavailable_reason}); no se manda nada para no "
                "bloquear el socket del brazo. close() (o reiniciar el nodo) "
                "permite reintentar."
            )
        error_code, value = self._commands.query(command)
        if error_code != 0:
            name = command.split("(", 1)[0]
            self._unavailable_reason = f"{name} devolvió {error_code}"
            raise Cr5ProtocolError(
                f"{name} devolvió el código de error {error_code} -- "
                f"{what_failed}. La pinza queda marcada como no disponible "
                "hasta close()."
            )
        return value

    def _ensure_master(self) -> None:
        if self._master_index >= 0:
            return
        for command in ("SetToolMode(1)", f'SetTool485({_BAUD},"{_PARITY}",{_STOP_BITS})'):
            self._query(command, "no se pudo preparar el 485 de la brida")
        value = self._query(
            f'ModbusCreate("{_TOOL485_GATEWAY_IP}",{_TOOL485_GATEWAY_PORT},{_SLAVE_ID},1)',
            "no se pudo abrir el maestro Modbus sobre el 485 de la brida",
        )
        try:
            self._master_index = int(value)
        except ValueError as error:
            raise Cr5ProtocolError(
                f'ModbusCreate devolvió un índice no numérico: "{value}"'
            ) from error

    def _write_action(self, registers: List[int]) -> None:
        self._ensure_master()
        values = "{" + ",".join(str(v) for v in registers) + "}"
        self._query(
            f"SetHoldRegs({self._master_index},{_ACTION_ADDR},"
            f"{_REGISTER_COUNT},{values},U16)",
            "la pinza no aceptó la orden",
        )

    def activate(self) -> None:
        """Si la pinza ya está activada y sin fallo grave, NO hace nada: la
        activación abre y cierra los dedos de tope a tope y soltaría lo que
        tuviera agarrado. Mismo criterio que activate() del SDK oficial de
        Robotiq (robotiq/grippers: "AlreadyActive ... nothing was sent").

        Si no lo está, o tiene un fallo grave, reset + rACT=1: rACT necesita
        un FLANCO de subida, y ese mismo reset es el que borra los fallos
        graves (el SDK lo separa en recoverFromFault; aquí GripperPort solo
        tiene activate)."""
        state = self.get_state()
        if state.activated and state.fault_code < _FIRST_MAJOR_FAULT:
            return
        self._write_action(_pack_action(0, 0, 0, 0, 0))
        self._write_action(_pack_action(1, 0, 0, 0, 0))

    def set_opening(
        self, fraction: float, speed: int = _DEFAULT_SPEED, force: int = _DEFAULT_FORCE
    ) -> None:
        if not 0.0 <= fraction <= 1.0:
            raise ValueError(
                f"apertura fuera de rango [0,1]: {fraction} (0 = abierta, 1 = cerrada)"
            )
        self._write_action(
            _pack_action(1, 1, round(fraction * 255), speed, force)
        )

    def get_state(self) -> GripperState:
        self._ensure_master()
        value = self._query(
            f"GetHoldRegs({self._master_index},{_STATUS_ADDR},{_REGISTER_COUNT})",
            "la pinza no contestó. Un -1 aquí NO distingue 'no hay pinza' de "
            "'pregunté mal': comprobar alimentación (SetToolPower), slave id, "
            "parámetros de 485 y, si todo cuadra, si 485A/485B están cruzados "
            "en el cable",
        )
        return _parse_status([int(v) for v in value.split(",") if v.strip()])

    def close(self) -> None:
        # Quitar la marca de "no disponible": tras close(), la siguiente
        # llamada vuelve a intentarlo desde cero (maestro nuevo incluido).
        self._unavailable_reason = None
        if self._master_index < 0:
            return
        # Best-effort: si el cierre falla, el maestro queda abierto en el
        # controlador (son 5 como mucho), pero no hay nada mejor que hacer
        # desde aquí y no debe impedir el cierre del nodo.
        try:
            self._commands.query(f"ModbusClose({self._master_index})")
        except Cr5ProtocolError:
            pass
        self._master_index = -1
