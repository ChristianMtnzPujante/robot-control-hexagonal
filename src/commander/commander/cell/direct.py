"""Modo de ejecución DIRECTO: construye una `CellDescription` compilada
dentro de este mismo proceso -- los adaptadores se instancian aquí y se
manejan con un `Manipulator`, sin nodos ROS. Es el modo para probar rápido
y para la primera vez con hardware nuevo: todo en un proceso, fácil de
depurar. El modo ROS (sesiones de `Commander`) se construirá desde la
MISMA descripción (fase 2).

    with open_direct(compile_cell("mesa_cubo")) as cell:
        cell.manipulator.move_joints(cell.posture("pre_agarre"))
        cell.manipulator.pick("cubo", cell.scene.bodies["cubo"])
        cell.manipulator.place(cell.scene.objects["destino"])

En simulación levanta CoppeliaSim si hace falta y construye la escena
entera (robot, herramienta y cuerpos). En real NO mueve nada al abrir:
conecta, activa la pinza (si ya lo estaba, no hace nada) y, al salir --
también si algo falla --, cierra la pinza y des-energiza el robot.
"""

from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Iterator, Optional, Sequence, Tuple

from shared_kernel import JointConfiguration, JointPosition, KinematicsPort, Pose, Scene

from ..manipulation import GraspSettings, Manipulator
from .adapters import REAL_GRIPPERS, REAL_ROBOTS, SIM_GRIPPERS, tool_mount
from .description import CellDescription
from .elements import RobotModel
from .errors import InvalidCellError

_REAL_STEP_PAUSE_SECONDS = 0.15


def ask_on_keyboard(message: str) -> bool:
    """`confirm` por defecto en el robot real: nada baja hacia la mesa sin
    un "s"."""
    return input(f"{message}. ¿Seguir? [s/N] ").strip().lower() in ("s", "si", "sí")


class SimulationView:
    """Lectura del simulador para comprobar resultados (dónde ha quedado un
    cuerpo). Solo existe en simulación: en real, eso es percepción."""

    def __init__(self, sim):
        self._sim = sim

    def body_position(self, name: str) -> Tuple[float, float, float]:
        return tuple(self._sim.getObjectPosition(self._sim.getObject(f"/{name}"), -1))

    def body_pose(self, name: str) -> Pose:
        """Pose exacta (posición y orientación) del cuerpo `name`, en el
        marco del mundo, que es el de la base del robot (ver
        `coppeliasim_scene_builder`)."""
        x, y, z, qx, qy, qz, qw = self._sim.getObjectPose(self._sim.getObject(f"/{name}"), -1)
        return Pose(x, y, z, qx, qy, qz, qw)


@dataclass
class CellHandle:
    description: CellDescription
    manipulator: Manipulator
    simulation: Optional[SimulationView] = None

    @property
    def scene(self) -> Scene:
        return self.description.scene

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self.description.robot.model.joint_names

    def posture(self, name: str) -> JointConfiguration:
        postures = self.description.postures
        if name not in postures:
            raise InvalidCellError(f'no hay ninguna postura "{name}". Definidas: {", ".join(postures) or "ninguna"}')
        return configuration_from_degrees(self.joint_names, postures[name])


def configuration_from_degrees(joint_names: Sequence[str], degrees: Sequence[float]) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(joint, math.radians(value)) for joint, value in zip(joint_names, degrees)]
    ).value


def _posture_seeds(cell: CellDescription) -> Tuple[JointConfiguration, ...]:
    """Las posturas con nombre de la célula, como semillas de IK para
    `Manipulator.move_to_pose`."""
    joints = cell.robot.model.joint_names
    return tuple(configuration_from_degrees(joints, degrees) for degrees in cell.postures.values())


def build_kinematics(kinematics: str, model: RobotModel) -> KinematicsPort:
    """La cinemática pedida, construida del URDF del modelo (no del CR5
    fijo que traen los adaptadores por defecto)."""
    from urdf_kit import parse_urdf_file

    description = parse_urdf_file(model.urdf_path, model.base_link, model.tip_link)
    if kinematics == "ga":
        # Solo aquí: necesita pygafro, que PoE no.
        from controller_node.adapters.ga_adapter import GaKinematicsAdapter

        return GaKinematicsAdapter(robot_description=description)
    from controller_node.adapters.poe_adapter import PoeKinematicsAdapter

    return PoeKinematicsAdapter(robot_description=description)


@contextmanager
def open_direct(
    cell: CellDescription,
    confirm: Optional[Callable[[str], bool]] = None,
    log: Callable[[str], None] = print,
) -> Iterator[CellHandle]:
    """`confirm` se pregunta antes de cada bajada hacia la mesa. Por
    defecto: siempre sí en simulación, por teclado en real."""
    opener = _open_sim if cell.robot.target == "sim" else _open_real
    with opener(cell, confirm, log) as handle:
        yield handle


@contextmanager
def _open_sim(cell: CellDescription, confirm, log) -> Iterator[CellHandle]:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    from ..coppeliasim_scene_builder import build_scene, ensure_coppeliasim_running

    model = cell.robot.model
    if not model.sim_root_alias:
        raise InvalidCellError(f'el robot "{model.name}" no tiene sección "sim": no se puede simular')
    tool = cell.tool
    if tool is not None and not tool.model.sim_adapter:
        raise InvalidCellError(f'la herramienta "{tool.model.name}" no tiene sección "sim"')
    port = cell.simulator.port
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cell_{port}")
    initial_degrees = (
        cell.postures[cell.robot.initial_posture] if cell.robot.initial_posture else [0.0] * len(model.joint_names)
    )
    robot = build_scene(
        port=port,
        urdf_path=model.urdf_path,
        urdf_package_prefix=model.package_prefix,
        joint_names=list(model.joint_names),
        tip_name=model.sim_tip_object,
        root_link_visual_alias=model.sim_root_alias,
        initial_configuration=configuration_from_degrees(model.joint_names, initial_degrees),
        scene=cell.scene,
        mounts=[tool_mount(tool.model, model)] if tool else [],
    )
    gripper = SIM_GRIPPERS[tool.model.sim_adapter].build(tool.model, port, cell.scene) if tool else None
    manipulator = Manipulator(
        robot,
        gripper,
        build_kinematics(cell.kinematics, model),
        GraspSettings(grasp_offset=tool.effective_grasp_offset if tool else 0.0, gripper_poll_seconds=0.05),
        confirm=confirm or (lambda message: True),
        step_pause_seconds=cell.simulator.step_pause_seconds,
        log=log,
        ik_seeds=_posture_seeds(cell),
    )
    sim = RemoteAPIClient(port=port).require("sim")
    try:
        yield CellHandle(cell, manipulator, SimulationView(sim))
    finally:
        if gripper is not None:
            gripper.close()
        robot.close()


@contextmanager
def _open_real(cell: CellDescription, confirm, log) -> Iterator[CellHandle]:
    model = cell.robot.model
    tool = cell.tool
    robot = REAL_ROBOTS[model.real_adapter].build(model, cell.robot.host, cell.robot.speed_factor)
    gripper = None
    try:
        if tool is not None:
            gripper = REAL_GRIPPERS[tool.model.real_adapter].build(tool.model, robot)
            gripper.activate()

        def wait_until_idle() -> None:
            if not robot.wait_until_idle():
                log("Aviso: el robot no volvió a reposo (RobotMode 5) a tiempo.")

        yield CellHandle(
            cell,
            Manipulator(
                robot,
                gripper,
                build_kinematics(cell.kinematics, model),
                GraspSettings(grasp_offset=tool.effective_grasp_offset if tool else 0.0),
                wait_until_idle=wait_until_idle,
                confirm=confirm or ask_on_keyboard,
                step_pause_seconds=_REAL_STEP_PAUSE_SECONDS,
                log=log,
                ik_seeds=_posture_seeds(cell),
            ),
        )
    finally:
        if gripper is not None:
            gripper.close()
        # close() ya des-energiza si el robot estaba habilitado.
        robot.close()
