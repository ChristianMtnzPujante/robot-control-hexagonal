"""Primera secuencia brazo + pinza: SUBIR el TCP unos centímetros en Z y,
cuando el robot ha terminado de moverse, ABRIR y CERRAR la pinza Robotiq 2F.

Mismo patrón que poe_lift_and_wrist_demo.py (del que reutiliza la lectura
de la postura real, la trayectoria de PoE y la espera a RobotMode()==5):
composición directa en Python contra los puertos del dominio, sin
ControlSession/controller_node -- el pipeline ROS2 todavía no sabe nada de
pinzas. Lo que es nuevo aquí es encadenar RobotConnectorPort y GripperPort
sobre la MISMA conexión TCP: el 29999 admite un solo cliente, así que la
pinza usa `robot.command_socket` (igual que robot_node._build_gripper).

Por qué esperar a RobotMode()==5 antes de tocar la pinza: el socket es
secuencial, pero MovJ solo ENCOLA el movimiento. Sin la espera, la pinza
empezaría a moverse con el brazo todavía subiendo.

Fases:
    --phase plan   calcula la trayectoria e informa (solo LEE la postura
                   real por el 30004; no mueve nada)
    --phase sim    reproduce la subida en CoppeliaSim con la 2F-85 montada
                   en la brida, y abre/cierra la pinza simulada
    --phase real   brazo real + pinza real

Uso:
    ros2 run commander lift_and_grip_demo --phase plan --host 192.168.5.1
    ros2 run commander lift_and_grip_demo --phase real --host 192.168.5.1

    Opcional: --lift-meters (por defecto 0.05 = subir 5 cm; negativo =
    bajar), --waypoint-pause-seconds (0.15).

Aviso: desde la home (todos los joints a 0) NO se puede subir -- ahí el
TCP está en el borde del alcance (ver cr5_semicircle_sim_demo.py, "la home
no puede subir, solo bajar"). Si PoE no converge, se aborta en la fase de
cálculo, antes de conectar para mover nada.

Pinza: activate() no hace nada si ya está activada (ver
robotiq_2f_adapter.py); si no lo está, la activación abre y cierra los
dedos de tope a tope.
"""

from __future__ import annotations

import argparse
import time

from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter
from robot_node.adapters.robotiq_2f_adapter import Robotiq2FGripperAdapter
from shared_kernel import GripperPort, Scene

from .cell import load_robot, load_tool
from .cell.adapters import SIM_GRIPPERS, tool_mount
from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

from .poe_lift_and_wrist_demo import (
    _JOINT_NAMES,
    _build_combined_trajectory,
    _format_degrees,
    _print_summary,
    _ZMQ_PORT,
    _read_real_current_configuration,
    _wait_until_robot_idle,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", required=True, choices=["plan", "sim", "real"])
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--lift-meters", type=float, default=0.05)
    parser.add_argument("--waypoint-pause-seconds", type=float, default=0.15)
    parser.add_argument(
        "--gripper-timeout-seconds",
        type=float,
        default=6.0,
        help="Máximo a esperar a que la pinza termine cada movimiento.",
    )
    return parser.parse_args()


def _move_gripper(
    gripper: GripperPort, name: str, opening: float, timeout_seconds: float
) -> None:
    """set_opening() vuelve en cuanto la orden se acepta; aquí se espera a
    que la posición deje de cambiar (o a detectar objeto) para que la
    secuencia sea de verdad secuencial."""
    print(f"\nPinza: {name} (opening={opening:.1f})...")
    gripper.set_opening(opening)
    deadline = time.monotonic() + timeout_seconds
    previous = None
    stable_reads = 0
    state = gripper.get_state()
    while time.monotonic() < deadline:
        time.sleep(0.2)
        state = gripper.get_state()
        if state.holding_object:
            break
        if previous is not None and abs(state.opening - previous) < 0.005:
            stable_reads += 1
            if stable_reads >= 2:
                break
        else:
            stable_reads = 0
        previous = state.opening
    else:
        print(f"  Aviso: no se estabilizó en {timeout_seconds}s.")
    print(
        f"  opening={state.opening:.2f}  objeto={'SÍ' if state.holding_object else 'no'}"
        f"  fallo=0x{state.fault_code:02X}"
    )


def _run_sim_phase(args: argparse.Namespace, current, combined) -> None:
    """Misma secuencia que la fase real, contra CoppeliaSim: el CR5 desde la
    postura real actual con la 2F-85 en la brida (`descriptions/tools/`),
    y la pinza por `GripperPort` igual que en la real."""
    print("\n=== FASE SIMULACIÓN -- CR5 + Robotiq 2F-85 en CoppeliaSim ===")
    ensure_coppeliasim_running(port=_ZMQ_PORT, settings_suffix="_lift_and_grip_demo")
    tool = load_tool("robotiq_2f_85")
    robot = build_cr5_scene(
        port=_ZMQ_PORT,
        initial_configuration=current,
        scene=Scene.empty(),
        mounts=[tool_mount(tool, load_robot("cr5"))],
    )
    gripper = SIM_GRIPPERS[tool.sim_adapter](tool, _ZMQ_PORT, Scene.empty())
    print(f"\nBrazo: subiendo ({len(combined)} waypoints)...")
    for waypoint in combined:
        robot.set_joints(waypoint)
        time.sleep(args.waypoint_pause_seconds)
    gripper.activate()
    _move_gripper(gripper, "ABRIR", 0.0, args.gripper_timeout_seconds)
    _move_gripper(gripper, "CERRAR", 1.0, args.gripper_timeout_seconds)
    print("\nSimulación terminada.")


def _run_real_phase(args: argparse.Namespace, combined) -> None:
    print("\n=== FASE REAL -- esto mueve el robot físico Y la pinza ===")
    robot = Cr5RealRobotAdapter(args.host, joint_names=_JOINT_NAMES)
    gripper = Robotiq2FGripperAdapter(robot.command_socket)
    try:
        print("\nPinza: comprobando que responde antes de mover el brazo...")
        state = gripper.get_state()
        print(
            f"  activada={state.activated}  opening={state.opening:.2f}"
            f"  fallo=0x{state.fault_code:02X}"
        )

        print(f"\nBrazo: subiendo ({len(combined)} waypoints)...")
        for waypoint in combined:
            robot.set_joints(waypoint)
            time.sleep(args.waypoint_pause_seconds)
        _wait_until_robot_idle(robot)
        print("Brazo: movimiento terminado.")

        gripper.activate()
        _move_gripper(gripper, "ABRIR", 0.0, args.gripper_timeout_seconds)
        _move_gripper(gripper, "CERRAR", 1.0, args.gripper_timeout_seconds)
    finally:
        gripper.close()
        # close() ya des-energiza sola si is_enabled (ver su docstring).
        robot.close()
    print("\nSecuencia terminada. Sesión real cerrada.")


def main() -> None:
    args = _parse_args()

    current = _read_real_current_configuration(args.host)
    print(f"Posición real actual: {_format_degrees(current)}")

    combined = _build_combined_trajectory(current, args.lift_meters, 0.0)
    print(
        f"Objetivo: {'subir' if args.lift_meters > 0 else 'bajar'} "
        f"{abs(args.lift_meters) * 100:.0f}cm en Z, después abrir y cerrar la pinza."
    )
    _print_summary(current, combined)

    if args.phase == "plan":
        print("\n(--phase plan: no se ha movido nada)")
    elif args.phase == "sim":
        _run_sim_phase(args, current, combined)
    else:
        _run_real_phase(args, combined)


if __name__ == "__main__":
    main()
