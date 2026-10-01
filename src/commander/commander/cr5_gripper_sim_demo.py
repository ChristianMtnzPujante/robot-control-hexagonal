"""El CR5 con la Robotiq 2F-85 montada en la brida, en CoppeliaSim: lleva
el brazo a una postura de trabajo (pinza apuntando hacia abajo), abre y
cierra la pinza, y gira la muñeca para ver que la pinza va solidaria con
Link6.

Escena construida por código (`build_cr5_scene` con
la 2F-85 de `descriptions/tools/robotiq_2f_85.yaml`): el CR5 desde su URDF y la pinza desde el
suyo (assets/robotiq_2f_85/), colgada de joint6. La pinza se mueve con
`CoppeliaSimGripperAdapter`, el `GripperPort` de simulación -- el mismo
contrato que `Robotiq2FGripperAdapter` usa con la pinza real.

Uso: `ros2 run commander cr5_gripper_sim_demo` (lanza CoppeliaSim solo si
hace falta).

    Opcional: --work-degrees (6 valores, postura de trabajo), --cycles
    (abrir/cerrar, por defecto 2), --port (por defecto 23000).
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List

from shared_kernel import JointConfiguration, JointPosition, Scene, Trajectory

from .cell import load_robot, load_tool
from .cell.adapters import SIM_GRIPPERS, tool_mount
from .coppeliasim_scene_builder import (
    build_cr5_scene,
    ensure_coppeliasim_running,
)

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
# Codo arriba, muñeca doblada: la brida queda mirando hacia abajo, delante
# de la base -- la postura típica antes de coger algo de la mesa.
_DEFAULT_WORK_DEGREES = [0.0, -30.0, 100.0, 20.0, -90.0, 0.0]
_STEPS = 40
_STEP_PAUSE_SECONDS = 0.04


def _configuration(degrees: List[float]) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, math.radians(value)) for name, value in zip(_JOINT_NAMES, degrees)]
    ).value


def _animate(robot, start: JointConfiguration, target: JointConfiguration) -> None:
    for waypoint in Trajectory.straight_line(start, target, _STEPS).waypoints:
        robot.set_joints(waypoint)
        time.sleep(_STEP_PAUSE_SECONDS)


def run(work_degrees: List[float], cycles: int = 2, port: int = _ZMQ_PORT) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_gripper_sim_demo_{port}")

    home = _configuration([0.0] * 6)
    tool = load_tool("robotiq_2f_85")
    robot = build_cr5_scene(
        port=port,
        initial_configuration=home,
        scene=Scene.empty(),
        mounts=[tool_mount(tool, load_robot("cr5"))],
    )
    gripper = SIM_GRIPPERS[tool.sim_adapter](tool, port, Scene.empty())

    work = _configuration(work_degrees)
    print("Brazo: de home a la postura de trabajo...")
    _animate(robot, home, work)

    for cycle in range(1, cycles + 1):
        for name, opening in (("ABRIR", 0.0), ("CERRAR", 1.0)):
            gripper.set_opening(opening)
            print(f"  ciclo {cycle}: {name} -> opening={gripper.get_state().opening:.2f}")
            time.sleep(0.3)

    print("Muñeca: girando joint6 90° con la pinza cerrada...")
    turned = _configuration(work_degrees[:5] + [work_degrees[5] + 90.0])
    _animate(robot, work, turned)
    gripper.set_opening(0.0)
    print(f"Pinza abierta de nuevo: opening={gripper.get_state().opening:.2f}")
    print("Listo. La escena sigue abierta en CoppeliaSim.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-degrees", nargs=6, type=float, default=_DEFAULT_WORK_DEGREES)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    args = parser.parse_args()
    run(args.work_degrees, cycles=args.cycles, port=args.port)


if __name__ == "__main__":
    main()
