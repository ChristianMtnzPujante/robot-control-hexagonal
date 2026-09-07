"""Combina dos movimientos en una sola trayectoria real: bajar el TCP en
Z (PoE resuelve la IK para el desplazamiento cartesiano, como
poe_sim_then_real_demo.py) y girar joint6 un ángulo fijo -- sumado en
espacio de articulaciones, LINEALMENTE a lo largo de los mismos waypoints
que PoE ya calcula para el descenso, no como un salto aparte al final.

Bypass deliberado de ControlSession/controller_node/rclpy: la estrategia
"poe" del pipeline ROS2 solo resuelve un objetivo cartesiano (Pose) -- no
hay forma de pedirle "además, gira este joint X grados" por ese canal.
Aquí se construye la trayectoria combinada directamente en Python (mismo
patrón de composición directa que avoid_obstacle_demo.py) y se manda
waypoint a waypoint contra RobotConnectorPort -- sin nodos ROS2 aparte,
así que tampoco hay riesgo de los procesos zombies de ControlSession
(#117): no se lanza ningún subproceso.

Por qué sumar joint6 EN EL WAYPOINT, no en la ORIENTACIÓN del objetivo
cartesiano: joint6 es el último eje de la muñeca (gira el propio efector
sobre su eje). Pedir ese giro como parte de la orientación del objetivo
dejaría que la IK decida cómo repartirlo entre los ejes de la muñeca, sin
garantía de que acabe siendo exactamente joint6. Sumarlo directamente en
espacio de articulaciones, sobre la solución de PoE para el descenso (que
en pruebas anteriores del mismo día deja joint6 prácticamente sin
cambio), consigue el giro exacto pedido sin ambigüedad.

Uso:
    ros2 run commander poe_lift_and_wrist_demo --phase sim --host <IP>
    ros2 run commander poe_lift_and_wrist_demo --phase real --host <IP>

    Opcional: --lift-meters (por defecto -0.06 = bajar 6cm; positivo =
    subir), --joint6-degrees (por defecto 45.0), --waypoint-pause-seconds
    (por defecto 0.15 -- pausa entre waypoints, análogo a
    waypoint_period_seconds en el pipeline ROS2).

CORREGIDO tras la primera prueba real: el `finally` des-energizaba el
robot casi inmediatamente después de mandar el último waypoint, sin
esperar a que terminara de llegar físicamente -- DisableRobot() corta el
movimiento donde esté en ese momento. Visto en vivo: los 6 joints
acabaron unos grados por detrás del objetivo, TODOS en la dirección del
movimiento (p. ej. joint6 pedido +45°, quedó en +40.86°) -- consistente
con un corte a mitad de camino, no con el suavizado de `cp` (que no
debería afectar al último punto de la cola). Ahora se espera a que
RobotMode() vuelva a 5 (habilitado e inactivo) antes de des-energizar.
"""

from __future__ import annotations

import argparse
import math
import time

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene

from robot_node.adapters._cr5_protocol import Cr5RealtimeSocket
from robot_node.adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter
from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", required=True, choices=["sim", "real"])
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--lift-meters", type=float, default=-0.06)
    parser.add_argument("--joint6-degrees", type=float, default=45.0)
    parser.add_argument("--waypoint-pause-seconds", type=float, default=0.15)
    return parser.parse_args()


def _read_real_current_configuration(host: str) -> JointConfiguration:
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


def _build_combined_trajectory(
    current: JointConfiguration, lift_meters: float, joint6_degrees: float
):
    """Trayectoria de PoE para el desplazamiento en Z, con joint6
    desplazado linealmente de 0 a `joint6_degrees` a lo largo de los
    MISMOS waypoints -- ver docstring del módulo sobre por qué aquí y no
    en el objetivo cartesiano."""
    poe = PoeKinematicsAdapter()
    current_pose = poe.forward_kinematics(current)
    goal = Pose(
        x=current_pose.x,
        y=current_pose.y,
        z=current_pose.z + lift_meters,
        qx=current_pose.qx,
        qy=current_pose.qy,
        qz=current_pose.qz,
        qw=current_pose.qw,
    )
    trajectory = poe.compute_trajectory(goal, current)
    waypoints = list(trajectory.waypoints)
    n = len(waypoints) - 1

    combined = []
    for i, waypoint in enumerate(waypoints):
        joint6_offset = math.radians(joint6_degrees) * (i / n)
        positions = [
            JointPosition(
                name,
                waypoint.angle_of(name) + (joint6_offset if name == "joint6" else 0.0),
            )
            for name in _JOINT_NAMES
        ]
        combined.append(JointConfiguration.create(positions).value)
    return combined


def _print_summary(current: JointConfiguration, combined) -> None:
    last = combined[-1]
    print(f"{len(combined)} waypoints. Deltas del último respecto al actual:")
    for name in _JOINT_NAMES:
        delta = math.degrees(last.angle_of(name) - current.angle_of(name))
        print(f"  {name}: {delta:+.2f}°")


def _run_sim_phase(args: argparse.Namespace, current: JointConfiguration, combined) -> None:
    print("\n=== FASE SIMULACIÓN -- arrancando CoppeliaSim desde la postura real actual ===")
    ensure_coppeliasim_running(port=_ZMQ_PORT, settings_suffix="_poe_lift_and_wrist")
    robot: CoppeliaSimRobotAdapter = build_cr5_scene(
        port=_ZMQ_PORT, initial_configuration=current, scene=Scene.empty()
    )
    for waypoint in combined:
        robot.set_joints(waypoint)
        time.sleep(args.waypoint_pause_seconds)
    print(
        "\nSimulación terminada. Revisa en CoppeliaSim que el movimiento sea el "
        "esperado antes de pedirme la fase real."
    )


def _wait_until_robot_idle(robot: Cr5RealRobotAdapter, timeout_seconds: float = 8.0) -> None:
    """Espera a que RobotMode() vuelva a 5 (habilitado e inactivo) antes
    de des-energizar -- ver la nota corregida en el docstring del módulo:
    sin esto, DisableRobot() puede cortar el movimiento a mitad de
    camino. Si se agota el timeout sin llegar a 5, sigue igualmente (no
    bloquear el cierre para siempre) -- close() des-energizará el estado
    en el que esté, que es mejor que no des-energizar nunca."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        mode, _ = robot.get_robot_mode()
        if mode == 5:
            return
        time.sleep(0.1)
    print(f"Aviso: RobotMode() no volvió a 5 en {timeout_seconds}s, cerrando igualmente.")


def _run_real_phase(args: argparse.Namespace, combined) -> None:
    print("\n=== FASE REAL -- esto mueve el robot físico ===")
    robot = Cr5RealRobotAdapter(args.host, joint_names=_JOINT_NAMES)
    try:
        for waypoint in combined:
            robot.set_joints(waypoint)
            time.sleep(args.waypoint_pause_seconds)
        print("\nTrayectoria enviada. Esperando a que el robot termine de moverse...")
        _wait_until_robot_idle(robot)
    finally:
        # close() ya des-energiza sola si is_enabled (ver su docstring).
        robot.close()
    print("Sesión real cerrada.")


def main() -> None:
    args = _parse_args()

    current = _read_real_current_configuration(args.host)
    print(f"Posición real actual: {_format_degrees(current)}")

    combined = _build_combined_trajectory(current, args.lift_meters, args.joint6_degrees)
    print(
        f"Objetivo: {'bajar' if args.lift_meters < 0 else 'subir'} "
        f"{abs(args.lift_meters) * 100:.0f}cm en Z, joint6 "
        f"{args.joint6_degrees:+.0f}° (repartido a lo largo de la trayectoria)."
    )
    _print_summary(current, combined)

    if args.phase == "sim":
        _run_sim_phase(args, current, combined)
    else:
        _run_real_phase(args, combined)


if __name__ == "__main__":
    main()
