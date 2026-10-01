"""Modo de ejecución DIRECTO: construye la célula descrita por una
`CellDescription` dentro de este mismo proceso -- los adaptadores se
instancian aquí y se manejan con un `Manipulator`, sin nodos ROS. Es el
modo para probar rápido y para la primera vez con hardware nuevo: todo en
un proceso, fácil de depurar. El modo ROS (sesiones de `Commander`) se
construirá desde la MISMA descripción (fase 2).

    with open_direct(load_cell("mesa_cubo")) as cell:
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
from typing import Callable, Iterator, Optional, Tuple

from shared_kernel import JointConfiguration, JointPosition, KinematicsPort, Scene

from ..manipulation import GraspSettings, Manipulator
from .catalog import RobotModel, robot_model, tool_model
from .description import CellDescription, InvalidCellError

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


@dataclass
class CellHandle:
    description: CellDescription
    manipulator: Manipulator
    joint_names: Tuple[str, ...]
    simulation: Optional[SimulationView] = None

    @property
    def scene(self) -> Scene:
        return self.description.scene

    def posture(self, name: str) -> JointConfiguration:
        if name not in self.description.postures:
            available = ", ".join(self.description.postures) or "ninguna"
            raise InvalidCellError(f'no hay ninguna postura "{name}". Definidas: {available}')
        return _configuration(self.joint_names, self.description.postures[name], name)


def _configuration(joint_names, degrees, name: str) -> JointConfiguration:
    if len(degrees) != len(joint_names):
        raise InvalidCellError(
            f'la postura "{name}" tiene {len(degrees)} valores y el robot {len(joint_names)} joints'
        )
    return JointConfiguration.create(
        [JointPosition(joint, math.radians(value)) for joint, value in zip(joint_names, degrees)]
    ).value


def build_kinematics(cell: CellDescription, model: RobotModel) -> KinematicsPort:
    """La cinemática pedida, construida del URDF del modelo (no del CR5
    fijo que traen los adaptadores por defecto)."""
    from urdf_kit import parse_urdf_file

    description = parse_urdf_file(model.urdf_path, model.base_link, model.tip_link)
    if cell.kinematics == "ga":
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
    model = robot_model(cell.robot.model)
    tool = cell.tool
    tool_entry = tool_model(tool.model, cell.robot.model) if tool else None
    opener = _open_sim if cell.robot.target == "sim" else _open_real
    with opener(cell, model, tool_entry, confirm, log) as handle:
        yield handle


@contextmanager
def _open_sim(cell, model, tool_entry, confirm, log) -> Iterator[CellHandle]:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    from ..coppeliasim_scene_builder import build_scene, ensure_coppeliasim_running

    port = cell.simulator.port
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cell_{port}")
    initial = (
        _configuration(model.joint_names, cell.postures[cell.robot.initial_posture], cell.robot.initial_posture)
        if cell.robot.initial_posture
        else _configuration(model.joint_names, [0.0] * len(model.joint_names), "home")
    )
    robot = build_scene(
        port=port,
        urdf_path=model.urdf_path,
        urdf_package_prefix=model.urdf_package_prefix,
        joint_names=list(model.joint_names),
        tip_name=model.sim_tip_name,
        root_link_visual_alias=model.root_link_visual_alias,
        initial_configuration=initial,
        scene=cell.scene,
        mounts=[tool_entry.mounts[cell.robot.model]] if tool_entry else [],
    )
    gripper = tool_entry.build_sim(port, cell.scene) if tool_entry else None
    grasp_offset = _grasp_offset(cell, tool_entry)
    manipulator = Manipulator(
        robot,
        gripper,
        build_kinematics(cell, model),
        GraspSettings(grasp_offset=grasp_offset, gripper_poll_seconds=0.05),
        confirm=confirm or (lambda message: True),
        step_pause_seconds=cell.simulator.step_pause_seconds,
        log=log,
    )
    sim = RemoteAPIClient(port=port).require("sim")
    try:
        yield CellHandle(cell, manipulator, model.joint_names, SimulationView(sim))
    finally:
        if gripper is not None:
            gripper.close()
        robot.close()


@contextmanager
def _open_real(cell, model, tool_entry, confirm, log) -> Iterator[CellHandle]:
    from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

    if cell.robot.model != "cr5":
        raise InvalidCellError(f'el modo real solo sabe conectar el CR5, no "{cell.robot.model}"')
    robot = Cr5RealRobotAdapter(cell.robot.host, joint_names=list(model.joint_names))
    gripper = None
    try:
        if tool_entry:
            gripper = tool_entry.build_real(robot)
            gripper.activate()

        def wait_until_idle() -> None:
            if not robot.wait_until_idle():
                log("Aviso: el robot no volvió a reposo (RobotMode 5) a tiempo.")

        yield CellHandle(
            cell,
            Manipulator(
                robot,
                gripper,
                build_kinematics(cell, model),
                GraspSettings(grasp_offset=_grasp_offset(cell, tool_entry)),
                wait_until_idle=wait_until_idle,
                confirm=confirm or ask_on_keyboard,
                step_pause_seconds=_REAL_STEP_PAUSE_SECONDS,
                log=log,
            ),
            model.joint_names,
        )
    finally:
        if gripper is not None:
            gripper.close()
        # close() ya des-energiza si el robot estaba habilitado.
        robot.close()


def _grasp_offset(cell: CellDescription, tool_entry) -> float:
    """El medido si lo hay; si no, el del modelo (solo llega aquí en
    simulación: `CellDescription.validate` lo exige en real). Sin
    herramienta no se usa."""
    tool = cell.tool
    if tool is None:
        return 0.0
    if tool.grasp_offset is not None:
        return tool.grasp_offset
    return tool_entry.sim_grasp_offset
