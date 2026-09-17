"""Verifica en CoppeliaSim, ANTES de tocar el CR5 físico, el mismo
movimiento que hará `cr5_go_home_demo.py` (`robot_node`) contra el robot
real: llevar los 6 joints a la configuración inicial (home, 0° -- la misma
que usa `PoeKinematicsAdapter` como origen) desde una postura de partida
distinta, para poder VER que el camino tiene sentido antes de arriesgar
hardware real.

No es el mismo código que el demo real -- ese habla contra
`Cr5RealRobotAdapter` (un único `MovJ`, el robot interpola solo); este
construye una escena en blanco con `coppeliasim_scene_builder.build_cr5_scene`
(mismo importador de URDF que `avoid_obstacle_demo.py`, sin depender de
`cr5_base.ttt`) y anima el trayecto en varios pasos con
`Trajectory.straight_line`, porque `CoppeliaSimRobotAdapter.set_joints`
teletransporta el joint al instante en vez de interpolar -- sin los pasos
intermedios, verías un salto en vez de un movimiento.

Uso: `ros2 run commander cr5_go_home_sim_demo` (lanza CoppeliaSim solo si
hace falta). Revisa en la ventana de CoppeliaSim que el brazo recorra un
camino razonable desde la postura de partida hasta la postura base
(vertical, todo a cero).

    Opcional: --start-degrees (6 valores, postura de partida -- por
    defecto una postura no trivial para que el movimiento se note),
    --target-degrees (por defecto la home real, 0 0 0 0 0 0), --steps
    (por defecto 30), --step-pause-seconds (por defecto 0.05), --port
    (por defecto 23000).
"""

from __future__ import annotations

import math
import time
from typing import List

from shared_kernel import JointConfiguration, JointPosition, Scene, Trajectory

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_DEFAULT_START_DEGREES = [30.0, -25.0, 40.0, 15.0, 20.0, -35.0]
_DEFAULT_TARGET_DEGREES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_DEFAULT_STEPS = 30
_DEFAULT_STEP_PAUSE_SECONDS = 0.05


def _parse_args():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-degrees", nargs=6, type=float, default=_DEFAULT_START_DEGREES)
    parser.add_argument("--target-degrees", nargs=6, type=float, default=_DEFAULT_TARGET_DEGREES)
    parser.add_argument("--steps", type=int, default=_DEFAULT_STEPS)
    parser.add_argument("--step-pause-seconds", type=float, default=_DEFAULT_STEP_PAUSE_SECONDS)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    return parser.parse_args()


def _configuration_from_degrees(joint_names: List[str], degrees: List[float]) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, math.radians(value)) for name, value in zip(joint_names, degrees)]
    ).value


def _format_degrees(joint_names: List[str], configuration: JointConfiguration) -> str:
    return ", ".join(
        f"{name}={math.degrees(configuration.angle_of(name)):.2f}°"
        for name in joint_names
    )


def run(
    start_configuration: JointConfiguration,
    target_configuration: JointConfiguration,
    steps: int = _DEFAULT_STEPS,
    step_pause_seconds: float = _DEFAULT_STEP_PAUSE_SECONDS,
    port: int = _ZMQ_PORT,
) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_go_home_sim_demo_{port}")

    robot = build_cr5_scene(
        port=port,
        initial_configuration=start_configuration,
        scene=Scene.empty(),
    )

    print(f"Postura de partida: {_format_degrees(_JOINT_NAMES, start_configuration)}")
    print(f"Objetivo (home):    {_format_degrees(_JOINT_NAMES, target_configuration)}")

    trajectory = Trajectory.straight_line(start_configuration, target_configuration, steps)
    print(f"Animando {len(trajectory.waypoints)} waypoints en CoppeliaSim...")
    for waypoint in trajectory.waypoints:
        robot.set_joints(waypoint)
        time.sleep(step_pause_seconds)

    final = trajectory.waypoints[-1]
    print(f"Listo. Posición final: {_format_degrees(_JOINT_NAMES, final)}")


def main() -> None:
    args = _parse_args()
    start_configuration = _configuration_from_degrees(_JOINT_NAMES, args.start_degrees)
    target_configuration = _configuration_from_degrees(_JOINT_NAMES, args.target_degrees)
    run(
        start_configuration,
        target_configuration,
        steps=args.steps,
        step_pause_seconds=args.step_pause_seconds,
        port=args.port,
    )


if __name__ == "__main__":
    main()
