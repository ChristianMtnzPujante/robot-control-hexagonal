"""Des-energiza el CR5 real de forma aislada, con una conexión fresca --
para cuando otro script se queda con el robot habilitado tras un fallo a
mitad de prueba (conexión de comandos muerta, ver
cr5_repeated_joint1_moves_demo.py) y no hay teach pendant ni DobotStudio
Pro a mano para apagarlo desde ahí.

Prueba DisableRobot() directamente primero, SIN RequestControl() previo --
hipótesis a confirmar en la práctica: el modo TCP es un estado del propio
robot (concedido la primera vez que algo lo pidió con éxito), no de la
conexión concreta, así que una conexión nueva no debería necesitar pedirlo
otra vez solo para apagar. Si eso falla, cae a intentar
RequestControl()+DisableRobot() como último recurso -- aunque el manual
dice que RequestControl() no está garantizado si el robot ya está
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


def main() -> None:
    args = _parse_args()
    commands = Cr5CommandSocket(args.host, args.port)
    commands.connect()

    _print_robot_mode(commands)

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
