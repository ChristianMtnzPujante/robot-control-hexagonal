"""Des-energiza el CR5 real de forma aislada, con una conexión fresca --
para cuando otro script se queda con el robot habilitado tras un fallo a
mitad de prueba (conexión de comandos muerta, ver
cr5_repeated_joint1_moves_demo.py) y no hay teach pendant ni DobotStudio
Pro a mano para apagarlo desde ahí.

CORREGIDO 08/09, tras un hallazgo real (`cr5_semicircle_demo.py`,
GetErrorID()=[76] -- "el extremo interfiere con el cuerpo del robot",
autocolisión leve): con el robot en estado de ALARMA, el manual oficial
(sección "Códigos de error generales") dice que NINGÚN comando de control
se ejecuta -- `DisableRobot()` devuelve -2 ("robot en estado de alarma")
hasta que se llama a `ClearError()` primero. Antes de esta corrección,
este script no lo intentaba, así que fallaba exactamente en el caso para
el que existe. `ClearError()` está en la lista de comandos permitidos en
estado de error/parada de emergencia del propio manual (junto a
`GetErrorID()`/`RobotMode()`/`Stop()`), así que es seguro intentarlo
siempre, haya alarma o no.

Prueba, en orden: (1) `GetErrorID()` para saber si hay alarma real: (2)
`ClearError()` si la hay -- limpia el flag, no mueve nada; (3)
`DisableRobot()` directamente, SIN `RequestControl()` previo -- hipótesis
confirmada en la práctica: el modo TCP es un estado del propio robot
(concedido la primera vez que algo lo pidió con éxito), no de la conexión
concreta, así que una conexión nueva no debería necesitar pedirlo otra vez
solo para apagar; (4) si eso falla, cae a intentar
`RequestControl()+DisableRobot()` como último recurso -- aunque el manual
dice que `RequestControl()` no está garantizado si el robot ya está
"habilitado e inactivo", así que puede que tampoco funcione; si ninguno de
los dos funciona, no queda más remedio que un teach pendant/DobotStudio
Pro, o cortar la alimentación de la caja de control físicamente.

Uso:
    ros2 run robot_node cr5_disable_demo --host <IP_DEL_ROBOT>
"""

from __future__ import annotations

import argparse

from robot_node.adapters._cr5_protocol import (
    DASHBOARD_PORT,
    ROBOT_MODE_DESCRIPTIONS,
    Cr5CommandSocket,
    Cr5ProtocolError,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--port", type=int, default=DASHBOARD_PORT)
    return parser.parse_args()


def _print_robot_mode(commands: Cr5CommandSocket) -> None:
    try:
        error_code, value = commands.query("RobotMode()")
    except Cr5ProtocolError as error:
        print(f"No se pudo consultar RobotMode(): {error}")
        return
    if error_code != 0:
        print(f"RobotMode() devolvió el código de error {error_code}")
        return
    mode = int(value)
    print(f"Estado actual: {mode} -- {ROBOT_MODE_DESCRIPTIONS.get(mode, 'código desconocido')}")


def _clear_error_if_any(commands: Cr5CommandSocket) -> None:
    """GetErrorID()/ClearError() están SIEMPRE permitidos (incluso en
    alarma/parada de emergencia, ver manual sección 5) -- seguro llamarlos
    de entrada, haya alarma real o no. No mueve nada, solo limpia el flag
    de alarma si la causa física ya no está presente."""
    try:
        error_code, value = commands.query("GetErrorID()")
    except Cr5ProtocolError as error:
        print(f"No se pudo consultar GetErrorID(): {error}")
        return
    if error_code != 0 or value in ("[]", ""):
        print(f"GetErrorID(): sin alarmas activas ({value!r}).")
        return
    print(f"GetErrorID(): alarma(s) activa(s) {value} -- intentando ClearError()...")
    try:
        clear_error_code = commands.send_command("ClearError()")
    except Cr5ProtocolError as error:
        print(f"ClearError() falló: {error}")
        return
    if clear_error_code != 0:
        print(f"ClearError() devolvió el código de error {clear_error_code}")
        return
    print("ClearError() OK -- comprobando que la alarma se limpió de verdad...")
    try:
        _, after = commands.query("GetErrorID()")
        print(f"GetErrorID() tras ClearError(): {after!r}")
    except Cr5ProtocolError as error:
        print(f"No se pudo reconsultar GetErrorID(): {error}")


def main() -> None:
    args = _parse_args()
    commands = Cr5CommandSocket(args.host, args.port)
    commands.connect()

    _print_robot_mode(commands)
    _clear_error_if_any(commands)

    print("\nIntentando DisableRobot() directamente (sin RequestControl previo)...")
    try:
        error_code = commands.send_command("DisableRobot()")
        if error_code == 0:
            print("Robot des-energizado correctamente.")
            commands.close()
            return
        print(f"DisableRobot() directo devolvió el código de error {error_code}.")
    except Cr5ProtocolError as error:
        print(f"DisableRobot() directo falló: {error}")
        print("Reconectando con una conexión completamente nueva...")
        commands.close()
        commands.connect()

    print("Probando con RequestControl() primero, luego DisableRobot()...")
    try:
        error_code = commands.send_command("RequestControl()")
        if error_code != 0:
            print(
                f"RequestControl() devolvió el código de error {error_code} -- "
                "no se puede pedir el control por TCP desde el estado actual."
            )
            print(
                "Sin teach pendant/DobotStudio Pro a mano, la única alternativa "
                "que queda es cortar la alimentación de la caja de control "
                "físicamente."
            )
            return
        error_code = commands.send_command("DisableRobot()")
        if error_code != 0:
            print(f"DisableRobot() (tras RequestControl) devolvió el código de error {error_code}.")
            return
        print("Robot des-energizado correctamente (tras RequestControl).")
    except Cr5ProtocolError as error:
        print(f"Fallo de protocolo/conexión: {error}")
        print(
            "Sin teach pendant/DobotStudio Pro a mano, la única alternativa que "
            "queda es cortar la alimentación de la caja de control físicamente."
        )
    finally:
        commands.close()


if __name__ == "__main__":
    main()
