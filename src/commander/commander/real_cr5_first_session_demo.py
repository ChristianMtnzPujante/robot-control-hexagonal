"""Primera vez que se prueba el camino REAL de la arquitectura contra el
CR5 físico -- Commander -> ControlSession -> controller_node -> robot_node
-> Cr5RealRobotAdapter -> robot. Hasta ahora el contacto con hardware real
(cr5_first_contact_demo.py y compañía, en robot_node) hablaba directo con
el adaptador, sin pasar por ninguno de estos nodos -- esto es justo lo que
faltaba (Vikunja Bloque 0 #110).

Estrategia "naive_test" a propósito, no PoE/GA: por diseño ignora el
objetivo cartesiano y solo aplica un barrido sinusoidal de amplitud
pequeña a la configuración ACTUAL de todos los joints a la vez -- un
único ciclo completo (fase 0 -> 2π), así que el último waypoint vuelve
exactamente a donde empezó. Es justo el "puerto de cinemática sencillo"
que hacía falta para esta prueba: valida el cableado end-to-end sin
arriesgar un salto cartesiano grande (PoE/GA sí podrían pedir un salto
articular grande si el objetivo está lejos, y todavía no hay validación
de límites en Cr5RealRobotAdapter -- ver Bloque 0 #114, deliberadamente
no resuelta hoy: la amplitud pequeña de esta prueba es la que sustituye
a esa validación por ahora, no un reemplazo permanente).

IMPORTANTE -- a diferencia de cr5_first_contact_demo.py, este script NO
pide confirmación por teclado (pensado para lanzarse con `ros2 run`, no
interactivo) y NO des-energiza el robot al terminar -- robot_node no
tiene ningún manejo de apagado al cerrarse (ver docstring de main() más
abajo). Ejecutar cr5_disable_demo.py después de esta prueba.

Uso:
    ros2 run commander real_cr5_first_session_demo --host <IP_DEL_ROBOT>

    Opcional: --amplitude-degrees (por defecto 2.0), --steps (por defecto
    6), --waypoint-period-seconds (por defecto 0.5).
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from ros2_kit import shutdown_node

from shared_kernel import Pose

from .commander_node import Commander

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--amplitude-degrees", type=float, default=2.0)
    parser.add_argument("--steps", type=int, default=6)
    parser.add_argument("--waypoint-period-seconds", type=float, default=0.5)
    return parser.parse_args()


def main(args=None) -> None:
    """robot_node no tiene NINGÚN manejo de cierre (ni señal, ni
    destroy_node propio) que llame a Cr5RealRobotAdapter.disable() -- al
    terminar `close_session` (SIGTERM al subproceso), el robot queda
    habilitado. Gap real, anotado pero no resuelto en esta prueba: usar
    cr5_disable_demo.py después de correr esto, igual que tras los demos
    de robot_node."""
    cli_args = _parse_args()

    rclpy.init(args=args)
    commander = Commander()

    total_seconds = cli_args.steps * cli_args.waypoint_period_seconds + 5.0
    commander.get_logger().info(
        f"Sesión real contra {cli_args.host}: amplitud="
        f"{cli_args.amplitude_degrees}°, steps={cli_args.steps}, "
        f"periodo={cli_args.waypoint_period_seconds}s -- corriendo "
        f"~{total_seconds:.1f}s en total."
    )

    commander.create_session(
        name="cr5_real",
        robot_target="real",
        controller_strategy="naive_test",
        joint_names=_JOINT_NAMES,
        waypoint_period_seconds=cli_args.waypoint_period_seconds,
        cr5_host=cli_args.host,
        naive_test_amplitude_radians=math.radians(cli_args.amplitude_degrees),
        naive_test_steps=cli_args.steps,
    )
    # Pose irrelevante: "naive_test" la ignora por completo (ver docstring
    # del módulo) -- solo hace falta publicar ALGO en /goal para disparar
    # _start_trajectory en controller_node.
    commander.send_goal("cr5_real", Pose(x=0.0, y=0.0, z=0.0))

    deadline = time.monotonic() + total_seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(commander, timeout_sec=0.2)
    finally:
        commander.get_logger().info("Cerrando la sesión...")
        commander.close_session("cr5_real")
        shutdown_node(commander)


if __name__ == "__main__":
    main()
