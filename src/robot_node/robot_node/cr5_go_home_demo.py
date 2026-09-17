"""Deja el CR5 físico en la configuración inicial ("home": los 6 joints a
0°, la que usa `PoeKinematicsAdapter` como origen -- ver
`controller_node/adapters/poe_adapter.py`) -- pensado como paso previo a
una prueba real (p. ej. varios objetivos consecutivos en la misma sesión,
ver Bloque 0), para arrancar siempre desde una postura conocida y
reproducible en vez de "donde sea que el robot se quedó la última vez".

Habla directo contra RobotConnectorPort/Cr5RealRobotAdapter, sin pasar por
robot_node ni por topics (mismo criterio que cr5_first_contact_demo.py y
compañía): esto es un movimiento de preparación, no algo que necesite el
resto del stack ROS2 en marcha todavía.

A diferencia de los demos de un único joint, este mueve los 6 a la vez en
un único MovJ -- el robot ya sabe interpolar internamente entre su postura
actual y el objetivo, no hace falta trocearlo aquí. Por eso mismo el
recorrido de algún joint puede ser bastante mayor que los 1-5° de esos
otros demos: se muestra el delta de CADA joint antes de pedir
confirmación, y se avisa explícitamente si alguno supera
`--warn-threshold-degrees` (por defecto 30°). MovJ se manda sin `v`/`a`
explícitos (usa la velocidad/aceleración por defecto del propio robot --
Cr5RealRobotAdapter todavía no expone reducirlas, ver Bloque 0 #114).

Des-energiza el robot al terminar (con éxito o con error) -- igual que
cr5_first_contact_demo.py -- para dejarlo listo para que la siguiente
sesión (p. ej. real_cr5_first_session_demo.py o una prueba de varios
objetivos por controller_node/Commander) pueda pedir RequestControl() +
EnableRobot() desde cero sin chocar con la restricción del manual de que
RequestControl() no está garantizado si el robot ya está habilitado.

Uso:
    ros2 run robot_node cr5_go_home_demo --host <IP_DEL_ROBOT>

    Opcional: --joint-names (si difieren de los de robot_node.yaml),
    --target-degrees (6 valores; por defecto 0 0 0 0 0 0, la home real),
    --warn-threshold-degrees (por defecto 30.0).
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List

from shared_kernel import JointConfiguration, JointPosition

from robot_node.adapters._cr5_protocol import CONTROLLABLE_ROBOT_MODES, Cr5ProtocolError
from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

_DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_DEFAULT_TARGET_DEGREES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_DEFAULT_WARN_THRESHOLD_DEGREES = 30.0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        required=True,
        help="IP del controlador del CR5 (la que se le asignó al puerto Ethernet).",
    )
    parser.add_argument(
        "--joint-names",
        nargs=6,
        default=_DEFAULT_JOINT_NAMES,
        help="6 nombres de joint, en el mismo orden que robot_node.yaml.",
    )
    parser.add_argument(
        "--target-degrees",
        nargs=6,
        type=float,
        default=_DEFAULT_TARGET_DEGREES,
        help="Configuración objetivo, en grados, en el mismo orden que "
        "--joint-names -- por defecto la home real (0 0 0 0 0 0).",
    )
    parser.add_argument(
        "--warn-threshold-degrees",
        type=float,
        default=_DEFAULT_WARN_THRESHOLD_DEGREES,
        help="Si algún joint tiene que recorrer más que esto, se muestra "
        "un aviso extra antes de pedir confirmación.",
    )
    return parser.parse_args()


def _format_degrees(joint_names: List[str], configuration: JointConfiguration) -> str:
    return ", ".join(
        f"{name}={math.degrees(configuration.angle_of(name)):.2f}°"
        for name in joint_names
    )


def _confirm(prompt: str) -> bool:
    answer = input(f"{prompt} Escribe SI (en mayúsculas) para continuar: ")
    return answer.strip() == "SI"


def main() -> None:
    args = _parse_args()
    joint_names = list(args.joint_names)
    target = JointConfiguration.create(
        [
            JointPosition(name, math.radians(degrees))
            for name, degrees in zip(joint_names, args.target_degrees)
        ]
    ).value

    print("=" * 70)
    print("CR5 FÍSICO -- volver a la configuración inicial ('home').")
    print(f"Host: {args.host}   Objetivo: {_format_degrees(joint_names, target)}")
    print("Ten una mano cerca del botón físico de parada de emergencia.")
    print("=" * 70)

    adapter = Cr5RealRobotAdapter(args.host, joint_names=joint_names)
    try:
        print("\nPaso 1/4 -- consultando el estado del robot (RobotMode)...")
        mode, description = adapter.get_robot_mode()
        print(f"Estado actual: {mode} -- {description}")
        if mode not in CONTROLLABLE_ROBOT_MODES:
            print(
                "\nSegún el manual, el robot NO está en un estado desde el que "
                "se garantice poder pedir el control por TCP (solo lo cubre "
                "apagado o des-energizado)."
            )
            if not _confirm("¿Quieres intentar continuar de todas formas?"):
                print("Cancelado por el usuario. No se ha mandado ningún comando de control.")
                return

        print("\nPaso 2/4 -- leyendo la posición actual (no mueve nada)...")
        current = adapter.get_current_configuration()
        print(f"Posición actual: {_format_degrees(joint_names, current)}")

        deltas_degrees = [
            math.degrees(target.angle_of(name) - current.angle_of(name))
            for name in joint_names
        ]
        print("\nRecorrido por joint hasta la configuración inicial:")
        for name, delta in zip(joint_names, deltas_degrees):
            marker = " <- MÁS DE LO HABITUAL" if abs(delta) > args.warn_threshold_degrees else ""
            print(f"  {name}: {delta:+.2f}°{marker}")
        if any(abs(delta) > args.warn_threshold_degrees for delta in deltas_degrees):
            print(
                f"\nAl menos un joint recorre más de {args.warn_threshold_degrees:.0f}° -- "
                "MovJ usa la velocidad/aceleración por defecto del propio robot "
                "(todavía no se puede reducir desde aquí), así que ese recorrido "
                "puede notarse rápido. Confirma solo si la zona alrededor del "
                "robot está despejada."
            )

        if not _confirm("\nPaso 3/4 -- ¿confirmas ESTE movimiento a home, ahora mismo?"):
            print("Cancelado por el usuario. No se ha mandado ningún movimiento.")
            return

        adapter.set_joints(target)
        print("Comando MovJ enviado. Esperando a que el robot llegue...")
        time.sleep(3.0)

        print("\nPaso 4/4 -- releyendo la posición para confirmar el resultado...")
        after = adapter.get_current_configuration()
        print(f"Posición final: {_format_degrees(joint_names, after)}")
        for name in joint_names:
            print(
                f"  {name}: esperado {math.degrees(target.angle_of(name)):.2f}°, "
                f"real {math.degrees(after.angle_of(name)):.2f}°"
            )
    except Cr5ProtocolError as error:
        print(f"\nFallo de protocolo/conexión: {error}")
        print("No se ha podido confirmar el movimiento -- revisa red/puertos antes de reintentar.")
    finally:
        if adapter.is_enabled:
            print(
                "\nDes-energizando el robot (DisableRobot) antes de salir -- así la "
                "siguiente sesión puede pedir RequestControl()/EnableRobot() desde "
                "cero sin chocar con la restricción del manual..."
            )
            try:
                adapter.disable()
                print("Robot des-energizado correctamente.")
            except Cr5ProtocolError as disable_error:
                print(f"No se pudo des-energizar automáticamente: {disable_error}")
                print(
                    "Des-energízalo a mano desde el panel/teach pendant antes de "
                    "dejar el robot sin vigilancia."
                )
        adapter.close()


if __name__ == "__main__":
    main()
