"""Montaje de una célula de trabajo: el CR5 con la Robotiq 2F-85, en
CoppeliaSim o en el robot real, devuelta como un `Manipulator` (ver
`manipulation.py`) listo para `pick`/`place`. Es lo único que cambia entre
simulación y real en un script de prueba.

Uso típico:

    with sim_workcell(scene) as manipulator:      # o real_workcell(host, settings)
        manipulator.move_joints(postura_de_trabajo)
        manipulator.pick("cubo", scene.bodies["cubo"])
        manipulator.place(Point(...))
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import Callable, Iterator, Optional

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import Scene

from .manipulation import GraspSettings, Manipulator

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def ask_on_keyboard(message: str) -> bool:
    """`confirm` para el robot real: nada baja hacia la mesa sin un "s"."""
    return input(f"{message}. ¿Seguir? [s/N] ").strip().lower() in ("s", "si", "sí")


@contextmanager
def sim_workcell(
    scene: Scene,
    settings: Optional[GraspSettings] = None,
    port: int = 23000,
    step_pause_seconds: float = 0.04,
) -> Iterator[Manipulator]:
    """El CR5 + 2F-85 en CoppeliaSim, con los cuerpos de `scene` dibujados y
    los `graspable` agarrables (agarre cinemático, ver
    `CoppeliaSimGripperAdapter`). Sin `settings`, usa la distancia de agarre
    del modelo (`ROBOTIQ_2F_85_GRASP_DEPTH`). El brazo empieza en home."""
    from shared_kernel import JointConfiguration, JointPosition

    from .coppeliasim_scene_builder import (
        ROBOTIQ_2F_85_GRASP_DEPTH,
        ROBOTIQ_2F_85_ON_CR5,
        build_cr5_scene,
        ensure_coppeliasim_running,
        robotiq_2f_85_gripper,
    )

    ensure_coppeliasim_running(port=port, settings_suffix=f"_workcell_{port}")
    home = JointConfiguration.create([JointPosition(name, 0.0) for name in _JOINT_NAMES]).value
    robot = build_cr5_scene(
        port=port, initial_configuration=home, scene=scene, mounts=[ROBOTIQ_2F_85_ON_CR5]
    )
    gripper = robotiq_2f_85_gripper(port, scene=scene)
    manipulator = Manipulator(
        robot,
        gripper,
        PoeKinematicsAdapter(),
        settings or GraspSettings(grasp_offset=ROBOTIQ_2F_85_GRASP_DEPTH, gripper_poll_seconds=0.05),
        step_pause_seconds=step_pause_seconds,
    )
    try:
        yield manipulator
    finally:
        gripper.close()
        robot.close()


@contextmanager
def real_workcell(
    host: str,
    settings: GraspSettings,
    confirm: Callable[[str], bool] = ask_on_keyboard,
    step_pause_seconds: float = 0.15,
) -> Iterator[Manipulator]:
    """El CR5 real con la Robotiq por el 485 de la brida (misma conexión que
    el brazo: el 29999 admite un solo cliente). `settings` es OBLIGATORIO:
    `grasp_offset` depende de cómo está montada la pinza y hay que medirlo.
    Activa la pinza al empezar (no hace nada si ya lo estaba) y, al salir,
    cierra la pinza y des-energiza el robot, también si algo falla."""
    from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter
    from robot_node.adapters.robotiq_2f_adapter import Robotiq2FGripperAdapter

    from .poe_lift_and_wrist_demo import _wait_until_robot_idle

    robot = Cr5RealRobotAdapter(host, joint_names=_JOINT_NAMES)
    gripper = Robotiq2FGripperAdapter(robot.command_socket)
    try:
        gripper.activate()
        yield Manipulator(
            robot,
            gripper,
            PoeKinematicsAdapter(),
            settings,
            wait_until_idle=lambda: _wait_until_robot_idle(robot),
            confirm=confirm,
            step_pause_seconds=step_pause_seconds,
        )
    finally:
        gripper.close()
        # close() ya des-energiza si el robot estaba habilitado.
        robot.close()
