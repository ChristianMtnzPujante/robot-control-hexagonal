"""Dos pruebas SENCILLAS de comparación PoE frente a GA (gafro) en
CoppeliaSim -- la versión mínima de `cr5_poe_vs_gafro_sim_demo.py`, sin
barridos aleatorios ni informes: un objetivo, dos adaptadores, y ver las
dos trayectorias una detrás de otra (rastro azul = PoE, verde = GA).

Prueba 1 -- "bajar 5 cm desde la home". La home tiene `joint5 = 0`, donde
los ejes de joint2/3/4/6 son paralelos (singularidad de muñeca): hay una
familia continua de soluciones y es esperable que PoE y GA lleguen a la
MISMA pose con articulaciones DISTINTAS. Se ve en la tabla: la punta
coincide, los ángulos no necesariamente.

Prueba 2 -- "desplazar 8 cm en -X y 5 cm en -Z desde una postura doblada"
(`joint2 = 30°, joint3 = -60°, joint5 = 40°`, lejos de la singularidad). Aquí
la solución es aislada: los dos Newton-Raphson deben dar la MISMA
configuración articular, no solo la misma pose.

Cada prueba: se resuelve la IK con los dos adaptadores (misma
configuración de partida, mismas tolerancias), se anima la trayectoria de
PoE, se vuelve al punto de partida, y se anima la de GA. Por consola: nº
de iteraciones, tiempo, configuración final de cada uno, diferencia
articular, y error real de la punta leído de CoppeliaSim al final de cada
trayectoria.

Uso: `ros2 run commander cr5_poe_vs_gafro_simple_demo` (lanza CoppeliaSim
si hace falta). Opcional: --step-pause-seconds (0.15), --port (23000).
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List

import numpy as np

from controller_node.adapters.ga_adapter import GaKinematicsAdapter
from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, KinematicsPort, Pose, Scene

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_TRAIL_COLORS = {"poe": [0.0, 0.4, 1.0], "ga": [0.0, 0.8, 0.2]}


def _configuration(values) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, float(value)) for name, value in zip(_JOINT_NAMES, values)]
    ).value


def _shifted(pose: Pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    """Misma orientación, posición desplazada."""
    return Pose(
        x=pose.x + dx, y=pose.y + dy, z=pose.z + dz,
        qx=pose.qx, qy=pose.qy, qz=pose.qz, qw=pose.qw,
    )


def _angles(configuration: JointConfiguration) -> np.ndarray:
    return np.array([configuration.angle_of(name) for name in _JOINT_NAMES])


def _run_case(
    title: str,
    robot,
    adapters: Dict[str, KinematicsPort],
    start: JointConfiguration,
    goal: Pose,
    pause: float,
) -> None:
    print(f"\n=== {title} ===")
    print(f"Objetivo: x={goal.x:.4f} y={goal.y:.4f} z={goal.z:.4f} (orientación fija)")
    robot.mark_goal(goal)

    finals: Dict[str, JointConfiguration] = {}
    for name, adapter in adapters.items():
        robot.set_trail_color(_TRAIL_COLORS[name])
        robot.set_joints(start)
        time.sleep(pause)

        started = time.perf_counter()
        trajectory = adapter.compute_trajectory(goal, start)
        elapsed_ms = 1e3 * (time.perf_counter() - started)

        for waypoint in trajectory.waypoints:
            robot.set_joints(waypoint)
            time.sleep(pause)
        finals[name] = trajectory.waypoints[-1]

        sim_tip = np.array(robot.tip_position())
        real_error_mm = 1e3 * np.linalg.norm(sim_tip - np.array([goal.x, goal.y, goal.z]))
        degrees = np.degrees(_angles(finals[name]))
        print(
            f"[{name.upper():3}] {getattr(adapter, 'last_iteration_count', '?'):>2} iteraciones, "
            f"{elapsed_ms:6.2f} ms, {len(trajectory.waypoints)} waypoints | "
            f"error real de la punta en CoppeliaSim: {real_error_mm:.3f} mm"
        )
        print(f"      articulaciones finales (°): {np.round(degrees, 2).tolist()}")

    difference = np.degrees(np.abs(_angles(finals["poe"]) - _angles(finals["ga"])))
    print(f"Diferencia articular PoE-GA (°): {np.round(difference, 2).tolist()} "
          f"-> máx {difference.max():.2f}°")
    if difference.max() < 0.5:
        print("=> Misma pose y MISMA configuración articular.")
    else:
        print("=> Misma pose, DISTINTA configuración articular (redundancia sin fijar).")


def run(step_pause_seconds: float, port: int) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_poe_vs_gafro_simple_demo_{port}")
    home = _configuration(np.zeros(6))
    robot = build_cr5_scene(port=port, initial_configuration=home, scene=Scene.empty())

    adapters: Dict[str, KinematicsPort] = {
        "poe": PoeKinematicsAdapter(),
        "ga": GaKinematicsAdapter(),
    }
    poe = adapters["poe"]

    # Prueba 1: bajar 5 cm desde la home (con joint5 = 0: muñeca singular).
    home_pose = poe.forward_kinematics(home)
    _run_case(
        "Prueba 1: bajar 5 cm desde la home",
        robot, adapters, home, _shifted(home_pose, dz=-0.05), step_pause_seconds,
    )

    # Prueba 2: desde una postura doblada (joint5 = 40°, lejos de la
    # singularidad), 8 cm en -X y 5 cm en -Z manteniendo la orientación.
    bent = _configuration(np.radians([0.0, 30.0, -60.0, 0.0, 40.0, 0.0]))
    bent_pose = poe.forward_kinematics(bent)
    _run_case(
        "Prueba 2: desde postura doblada, 8 cm en -X y 5 cm en -Z",
        robot, adapters, bent, _shifted(bent_pose, dx=-0.08, dz=-0.05), step_pause_seconds,
    )

    robot.set_trail_color(_TRAIL_COLORS["poe"])
    print("\nListo. Rastro azul = PoE, rastro verde = GA. El robot queda en el final de la prueba 2 (GA).")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-pause-seconds", type=float, default=0.15)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    args = parser.parse_args()
    run(step_pause_seconds=args.step_pause_seconds, port=args.port)


if __name__ == "__main__":
    main()
