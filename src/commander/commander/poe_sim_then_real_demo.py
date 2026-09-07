"""Primera prueba de cinemática PoE REAL (no naive_test) contra el CR5,
en dos fases -- simulación primero, robot físico después, a petición
explícita del usuario (07/09): "lanzando primero la simulación y luego
enviando los datos al robot real".

A diferencia de real_cr5_first_session_demo.py (que usaba "naive_test",
un doble de pruebas que ignora el objetivo cartesiano por completo), esto
usa PoeKinematicsAdapter de verdad: resuelve una IK real para un objetivo
cartesiano. Por eso hace falta más cuidado que con naive_test -- el salto
articular resultante depende del objetivo, no está acotado por diseño.

El objetivo se calcula relativo a la pose ACTUAL real del robot (leída por
el feed real-time, de solo lectura, sin riesgo): "subir --lift-meters en Z
manteniendo la orientación", no una pose absoluta fija -- así el objetivo
sigue siendo pequeño y razonable sea cual sea la postura de partida.

La fase de SIMULACIÓN construye la escena de CoppeliaSim arrancando desde
la MISMA configuración articular que tiene el robot real en este momento
(no desde la "home" de PoE) -- así la trayectoria que se ve en CoppeliaSim
es una previsualización fiel de la que se mandaría al robot real, no solo
una comprobación genérica de que PoE converge.

--phase real mueve el robot físico -- NO se ejecuta nunca automáticamente
tras --phase sim en el mismo proceso. Cada fase es una invocación aparte,
para que quede un punto de confirmación humana explícito entre las dos
(ver la instrucción del usuario del 07/09 sobre confirmar antes de mover
el robot real).

Uso:
    ros2 run commander poe_sim_then_real_demo --phase sim --host <IP>
    ros2 run commander poe_sim_then_real_demo --phase real --host <IP>

    Opcional: --lift-meters (por defecto 0.03 = 3cm), --waypoint-period-seconds.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from ros2_kit import shutdown_node

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene

from robot_node.adapters._cr5_protocol import Cr5CommandSocket, Cr5RealtimeSocket

from .commander_node import Commander
from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", required=True, choices=["sim", "real"])
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument(
        "--lift-meters",
        type=float,
        default=0.03,
        help="Cuánto subir el TCP en Z (misma orientación) desde la pose actual.",
    )
    parser.add_argument("--waypoint-period-seconds", type=float, default=0.3)
    return parser.parse_args()


def _read_real_current_configuration(host: str) -> JointConfiguration:
    """Solo lectura (puerto 30004, real-time) -- sin riesgo, no toca el
    socket de comandos ni energiza nada."""
    realtime = Cr5RealtimeSocket(host, timeout=6.0)
    realtime.connect()
    try:
        angles_deg = realtime.read_joint_angles_deg()
    finally:
        realtime.close()
    positions = [
        JointPosition(name, math.radians(angle))
        for name, angle in zip(_JOINT_NAMES, angles_deg)
    ]
    return JointConfiguration.create(positions).value


def _format_degrees(configuration: JointConfiguration) -> str:
    return ", ".join(
        f"{name}={math.degrees(configuration.angle_of(name)):.2f}°" for name in _JOINT_NAMES
    )


def _compute_goal(current: JointConfiguration, lift_meters: float) -> Pose:
    poe = PoeKinematicsAdapter()
    current_pose = poe.forward_kinematics(current)
    return Pose(
        x=current_pose.x,
        y=current_pose.y,
        z=current_pose.z + lift_meters,
        qx=current_pose.qx,
        qy=current_pose.qy,
        qz=current_pose.qz,
        qw=current_pose.qw,
    )


def _print_expected_trajectory(current: JointConfiguration, goal: Pose) -> None:
    poe = PoeKinematicsAdapter()
    trajectory = poe.compute_trajectory(goal, current)
    last = trajectory.waypoints[-1]
    deltas = [
        math.degrees(last.angle_of(name) - current.angle_of(name)) for name in _JOINT_NAMES
    ]
    print(f"PoE calcula {len(trajectory.waypoints)} waypoints. Deltas del último respecto al actual:")
    for name, delta in zip(_JOINT_NAMES, deltas):
        print(f"  {name}: {delta:+.2f}°")


def _run_sim_phase(args: argparse.Namespace, current: JointConfiguration, goal: Pose) -> None:
    print("\n=== FASE SIMULACIÓN -- arrancando CoppeliaSim desde la postura real actual ===")
    ensure_coppeliasim_running(port=_ZMQ_PORT, settings_suffix="_poe_sim_then_real")
    build_cr5_scene(port=_ZMQ_PORT, initial_configuration=current, scene=Scene.empty())

    rclpy.init()
    commander = Commander()
    try:
        commander.create_session(
            name="poe_sim",
            robot_target="simulado",
            controller_strategy="poe",
            joint_names=_JOINT_NAMES,
            waypoint_period_seconds=args.waypoint_period_seconds,
            zmq_port=_ZMQ_PORT,
        )
        commander.send_goal("poe_sim", goal)

        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(commander, timeout_sec=0.2)
    finally:
        commander.close_session("poe_sim")
        shutdown_node(commander)

    print(
        "\nSimulación terminada. Revisa en la ventana de CoppeliaSim que el "
        "movimiento sea el esperado (solo un pequeño ajuste, sin nada raro) "
        "antes de pedirme la fase real."
    )


def _wait_until_robot_idle(host: str, timeout_seconds: float = 20.0) -> None:
    """Espera a que RobotMode() vuelva a 5 (habilitado e inactivo) antes
    de cerrar la sesión.

    CORREGIDO tras una prueba real: sin esto, ControlSession.stop() (que
    desde el arreglo de #117 SÍ mata al robot_node real, no solo al
    lanzador de `ros2 run`) puede matar el proceso mientras todavía está
    procesando su cola de suscripción ROS2 -- los mensajes joint_command
    sin procesar en ese momento simplemente desaparecen con el proceso,
    SIN ningún error visible. Visto en vivo: 21 waypoints publicados y
    "completado" en el lado de controller_node, pero el robot real apenas
    se movió -- robot_node no había llegado ni de lejos a procesar toda
    la cola cuando el deadline fijo de 12s mató la sesión. El deadline
    fijo asumía el ritmo de CoppeliaSim (prácticamente instantáneo);
    contra hardware real, cada MovJ tarda un tiempo de red de verdad.

    Usa una conexión de comandos propia (no la de robot_node, que vive en
    otro proceso) -- RobotMode() es una consulta de solo lectura, no
    interfiere con quien tenga el control."""
    commands = Cr5CommandSocket(host, 29999, timeout=3.0)
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        try:
            _, value = commands.query("RobotMode()")
            if int(value) == 5:
                commands.close()
                return
        except Exception:
            pass
        time.sleep(0.3)
    commands.close()
    print(f"Aviso: el robot no volvió a RobotMode()=5 en {timeout_seconds}s -- cerrando igualmente.")


def _run_real_phase(args: argparse.Namespace, current: JointConfiguration, goal: Pose) -> None:
    print("\n=== FASE REAL -- esto mueve el robot físico ===")
    rclpy.init()
    commander = Commander()
    try:
        commander.create_session(
            name="poe_real",
            robot_target="real",
            controller_strategy="poe",
            joint_names=_JOINT_NAMES,
            waypoint_period_seconds=args.waypoint_period_seconds,
            cr5_host=args.host,
        )
        commander.send_goal("poe_real", goal)

        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(commander, timeout_sec=0.2)

        print("\nEsperando a que el robot termine de procesar la cola...")
        _wait_until_robot_idle(args.host)
    finally:
        commander.close_session("poe_real")
        shutdown_node(commander)

    print("\nSesión real cerrada. Comprueba la posición final con cr5_disable_demo.py si hace falta.")


def main() -> None:
    args = _parse_args()

    current = _read_real_current_configuration(args.host)
    print(f"Posición real actual: {_format_degrees(current)}")

    goal = _compute_goal(current, args.lift_meters)
    print(f"Objetivo: subir {args.lift_meters * 100:.0f}cm en Z manteniendo orientación.")
    _print_expected_trajectory(current, goal)

    if args.phase == "sim":
        _run_sim_phase(args, current, goal)
    else:
        _run_real_phase(args, current, goal)


if __name__ == "__main__":
    main()
