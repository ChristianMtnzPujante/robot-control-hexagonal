"""Primer contacto con un CR5 físico real -- prueba manual, NO un nodo ROS2
(habla directo contra RobotConnectorPort/Cr5RealRobotAdapter, sin pasar por
robot_node ni por topics) para aislar "¿el protocolo funciona con ESTE
robot?" de "¿el resto del stack ROS2 funciona?" -- son preguntas distintas
y esta solo responde la primera. Parte del checklist físico de Bloque 0
(#112) y paso previo a la primera validación real (#110).

Deliberadamente NO es un test automático: cada paso pide confirmación por
teclado y muestra qué se va a mandar ANTES de mandarlo, porque hay un robot
de verdad al otro lado y _cr5_protocol.py todavía no valida límites
articulares ni de velocidad (ver Bloque 0 #114) -- aquí la validación la
hace la persona delante del botón de emergencia, no el código.

Secuencia, en orden creciente de riesgo:
  1. Consultar RobotMode() -- de solo lectura, no requiere haber pedido el
     modo TCP todavía. Si el robot no está en un estado desde el que el
     manual garantiza poder pedir el modo TCP (des-energizado o apagado),
     se avisa y se PIDE CONFIRMACIÓN antes de seguir -- no se bloquea en
     seco, porque sin teach pendant ni DobotStudio Pro este TCP puede ser
     el único canal de control disponible, y el manual no documenta qué
     pasa exactamente al intentarlo desde otro estado (lo más plausible es
     un error limpio, no algo peor, pero no se sabía con certeza hasta
     probarlo). Sin este chequeo, la primera prueba real se encontró con
     la conexión reseteada al llegar a EnableRobot() (ver el "CORREGIDO
     04/09 (2)" en cr5_real_adapter.py).
  2. LEER la posición actual (get_current_configuration) -- riesgo cero,
     no manda ningún comando de movimiento.
  3. Mostrar esa lectura y pedir confirmación explícita antes de mover.
  4. Mover UN SOLO joint un ángulo pequeño (por defecto 3°) relativo a su
     posición ACTUAL leída en el paso 2 -- nunca un ángulo absoluto fijo,
     precisamente para que el movimiento sea corto sin importar en qué
     postura esté el robot al arrancar el script.
  5. Leer de nuevo y mostrar el resultado, para confirmar visualmente (en
     los números, y con los propios ojos sobre el robot) que se movió lo
     esperado y nada más.
  6. Des-energizar (DisableRobot()) antes de salir, SOLO si este mismo
     script llegó a habilitar el robot -- para no dejarlo con los servos
     activos sin nadie vigilando después de que el script termine. Corre
     tanto si el paso 4 tuvo éxito como si falló a mitad (ver el `finally`
     de main()); no corre si nunca se llegó a habilitar (nada que
     deshacer, y DisableRobot() exige el mismo modo TCP que EnableRobot).

Uso:
    ros2 run robot_node cr5_first_contact_demo --host <IP_DEL_ROBOT>

    Opcional: --joint-index (0-5, por defecto 0 -> joint1),
    --delta-degrees (por defecto 3.0), --joint-names (si difieren de los
    de robot_node.yaml).
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
    parser.add_argument(
        "--delta-degrees",
        type=float,
        default=3.0,
        help="Cuánto mover ese joint respecto a su posición actual, en grados.",
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

    print("=" * 70)
    print("PRIMER CONTACTO CON EL CR5 FÍSICO -- lee todo antes de confirmar nada.")
    print(f"Host: {args.host}   Joint a mover: {joint_names[args.joint_index]}"
          f"   Delta: {args.delta_degrees}°")
    print("Ten una mano cerca del botón físico de parada de emergencia.")
    print("=" * 70)

    adapter = Cr5RealRobotAdapter(args.host, joint_names=joint_names)
    try:
        print("\nPaso 1/5 -- consultando el estado del robot (RobotMode)...")
        mode, description = adapter.get_robot_mode()
        print(f"Estado actual: {mode} -- {description}")
        if mode not in CONTROLLABLE_ROBOT_MODES:
            print(
                "\nSegún el manual, el robot NO está en un estado desde el que "
                "se garantice poder pedir el control por TCP (solo lo cubre "
                "apagado o des-energizado) -- normalmente eso se arregla "
                "des-energizando desde el panel/teach pendant o DobotStudio "
                "Pro. Si no tienes ninguno de los dos, el manual no dice qué "
                "pasa exactamente al intentarlo desde este otro estado: puede "
                "que el robot simplemente devuelva un error limpio (en cuyo "
                "caso no pasa nada) sin que lo sepamos hasta intentarlo."
            )
            if not _confirm(
                "¿Quieres intentar continuar de todas formas para ver la "
                "respuesta real del robot? (esto NO manda ningún movimiento "
                "todavía, solo prueba RequestControl más adelante)"
            ):
                print("Cancelado por el usuario. No se ha mandado ningún comando de control.")
                return

        print("\nPaso 2/5 -- leyendo la posición actual (no mueve nada)...")
        current = adapter.get_current_configuration()
        print(f"Posición actual: {_format_degrees(joint_names, current)}")

        if not _confirm(
            "\nPaso 3/5 -- ¿la lectura de arriba tiene sentido "
            "(seis números creíbles, ni NaN ni todo ceros si el robot no "
            "está en home) y quieres continuar con el movimiento?"
        ):
            print("Cancelado por el usuario. No se ha mandado ningún movimiento.")
            return

        moving_joint = joint_names[args.joint_index]
        target_positions = [
            JointPosition(
                name,
                current.angle_of(name)
                + (math.radians(args.delta_degrees) if name == moving_joint else 0.0),
            )
            for name in joint_names
        ]
        target = JointConfiguration.create(target_positions).value

        print(
            f"\nSe va a mandar: {moving_joint} pasa de "
            f"{math.degrees(current.angle_of(moving_joint)):.2f}° a "
            f"{math.degrees(target.angle_of(moving_joint)):.2f}° "
            "(el resto de joints se manda sin cambios)."
        )
        if not _confirm("Paso 4/5 -- ¿confirmas ESTE movimiento, ahora mismo?"):
            print("Cancelado por el usuario. No se ha mandado ningún movimiento.")
            return

        adapter.set_joints(target)
        print("Comando MovJ enviado. Esperando a que el robot llegue...")
        time.sleep(2.0)

        print("\nPaso 5/5 -- releyendo la posición para confirmar el resultado...")
        after = adapter.get_current_configuration()
        print(f"Posición tras el movimiento: {_format_degrees(joint_names, after)}")
        print(
            f"{moving_joint}: esperado {math.degrees(target.angle_of(moving_joint)):.2f}°, "
            f"real {math.degrees(after.angle_of(moving_joint)):.2f}°"
        )
    except Cr5ProtocolError as error:
        print(f"\nFallo de protocolo/conexión: {error}")
        print("No se ha podido confirmar el movimiento -- revisa red/puertos antes de reintentar.")
    finally:
        if adapter.is_enabled:
            print("\nPaso final -- des-energizando el robot (DisableRobot) antes de salir...")
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
