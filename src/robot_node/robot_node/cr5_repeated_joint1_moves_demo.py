"""Prueba de CONEXIÓN PERSISTENTE contra el CR5 físico: en vez de un único
movimiento (ver cr5_first_contact_demo.py), manda varios comandos MovJ
seguidos por la MISMA conexión TCP, uno por cada número que se escriba en
terminal -- justo lo que hará falta para reproducir una trayectoria real
(varios waypoints seguidos), a diferencia de la prueba de un solo
movimiento que ya se validó.

No es un test automático, es una herramienta interactiva: solo mueve
joint1 (por defecto), solo hacia adelante o atrás en pasos de 1 a 5 grados
elegidos a mano, y cada línea escrita SE ENVÍA DIRECTAMENTE -- a propósito,
sin una confirmación "SI" extra por movimiento (a diferencia de
cr5_first_contact_demo.py), porque el rango 1-5° ya acota el riesgo de
cada paso individual y el objetivo aquí es poder encadenar varios
movimientos rápido para observar si la conexión aguanta. Sigue haciendo
falta una mano cerca del botón de emergencia.

Uso:
    ros2 run robot_node cr5_repeated_joint1_moves_demo --host <IP_DEL_ROBOT>

    Opcional: --joint-index (0-5, por defecto 0 -> joint1), --joint-names.
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
_MIN_STEP_DEGREES = 1
_MAX_STEP_DEGREES = 5


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
        "--joint-index",
        type=int,
        default=0,
        choices=range(6),
        help="Índice (0-5) del único joint que se moverá. 0 = joint1, la base.",
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
    moving_joint = joint_names[args.joint_index]

    print("=" * 70)
    print("PRUEBA DE CONEXIÓN PERSISTENTE CONTRA EL CR5 -- varios movimientos")
    print("seguidos por la misma conexión, no uno solo.")
    print(f"Host: {args.host}   Joint a mover: {moving_joint}")
    print("Ten una mano cerca del botón físico de parada de emergencia.")
    print("=" * 70)

    adapter = Cr5RealRobotAdapter(args.host, joint_names=joint_names)
    try:
        print("\nConsultando el estado del robot (RobotMode)...")
        mode, description = adapter.get_robot_mode()
        print(f"Estado actual: {mode} -- {description}")
        if mode not in CONTROLLABLE_ROBOT_MODES:
            print(
                "\nSegún el manual, el robot no está en un estado en el que se "
                "garantice poder pedir el control por TCP -- puede que el "
                "primer movimiento falle con un error limpio."
            )
            if not _confirm("¿Intentar de todas formas?"):
                print("Cancelado por el usuario.")
                return

        current = adapter.get_current_configuration()
        print(f"\nPosición actual: {_format_degrees(joint_names, current)}")

        print(
            f"\nEscribe un número entre {_MIN_STEP_DEGREES} y {_MAX_STEP_DEGREES} "
            f"(grados a mover {moving_joint} respecto a su posición actual) y "
            "pulsa Enter para mandarlo AL INSTANTE -- sin confirmación extra. "
            "Escribe 'q' para terminar."
        )

        move_count = 0
        while True:
            raw = input(f"\n[{moving_joint}] grados a mover (1-5, o 'q'): ").strip()
            if raw.lower() in ("q", "salir", ""):
                break
            try:
                degrees = int(raw)
            except ValueError:
                print(f'"{raw}" no es un número entero -- prueba otra vez.')
                continue
            if not (_MIN_STEP_DEGREES <= degrees <= _MAX_STEP_DEGREES):
                print(
                    f"Fuera de rango: tiene que estar entre {_MIN_STEP_DEGREES} "
                    f"y {_MAX_STEP_DEGREES}."
                )
                continue

            current = adapter.get_current_configuration()
            target_positions = [
                JointPosition(
                    name,
                    current.angle_of(name)
                    + (math.radians(degrees) if name == moving_joint else 0.0),
                )
                for name in joint_names
            ]
            target = JointConfiguration.create(target_positions).value

            print(
                f"Mandando: {moving_joint} de "
                f"{math.degrees(current.angle_of(moving_joint)):.2f}° a "
                f"{math.degrees(target.angle_of(moving_joint)):.2f}° "
                f"(movimiento #{move_count + 1} de esta misma conexión)..."
            )
            adapter.set_joints(target)
            move_count += 1
            time.sleep(1.0)
            after = adapter.get_current_configuration()
            print(f"Posición tras el movimiento: {_format_degrees(joint_names, after)}")

        print(f"\nTerminado -- {move_count} movimiento(s) mandados por la misma conexión.")
    except Cr5ProtocolError as error:
        print(f"\nFallo de protocolo/conexión: {error}")
    except KeyboardInterrupt:
        print("\nInterrumpido por teclado (Ctrl+C).")
    finally:
        if adapter.is_enabled:
            print("\nDes-energizando el robot (DisableRobot) antes de salir...")
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
