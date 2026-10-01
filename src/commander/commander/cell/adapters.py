"""Registro de adaptadores por nombre: lo que los YAML de `descriptions/`
citan como `adapter: <nombre>`. Es la única parte de una célula que es
código: un robot o una pinza que hable un protocolo nuevo necesita su
adaptador aquí; uno que solo cambie de medidas o de montaje, no.

Cada entrada declara además qué CAPACIDADES aporta (`capabilities.py`):
es una propiedad de la implementación, así que vive junto a su código.

Las importaciones van DENTRO de cada factoría: así compilar una célula
(que comprueba que los nombres existen) no necesita CoppeliaSim, ni el
robot, ni pygafro.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, FrozenSet

from shared_kernel import GripperPort, RobotConnectorPort, Scene

from . import capabilities as caps
from .elements import RobotModel, ToolModel


@dataclass(frozen=True)
class AdapterEntry:
    """`build` construye el adaptador; `provides` dice qué capacidades
    aporta para un modelo concreto (p. ej. una pinza simulada solo detecta
    el agarre si su modelo trae la geometría de las yemas)."""

    build: Callable
    provides: Callable[[object], FrozenSet[str]]
    doc: str


def _cr5_tcp(model: RobotModel, host: str, speed_factor=None) -> RobotConnectorPort:
    from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

    limits = list(model.real_joint_limits_degrees) if model.real_joint_limits_degrees else None
    return Cr5RealRobotAdapter(host, joint_names=list(model.joint_names), joint_limits_degrees=limits,
                               speed_factor=speed_factor)


def _coppeliasim_urdf_gripper(tool: ToolModel, port: int, scene: Scene) -> GripperPort:
    """Pinza importada de su URDF en CoppeliaSim, cinemática; con agarre
    cinemático de los cuerpos `graspable` de `scene` si el modelo trae la
    geometría de las yemas."""
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    from robot_node.adapters.coppeliasim_gripper_adapter import (
        CoppeliaSimGripperAdapter,
        GraspGeometry,
        gripper_joints_from_urdf,
    )

    grasp = None
    if tool.pads is not None:
        left_knuckle, right_knuckle, left_tip, right_tip = tool.pads.frame_joints
        grasp = GraspGeometry(
            left_knuckle_joint=left_knuckle,
            right_knuckle_joint=right_knuckle,
            left_tip_joint=left_tip,
            right_tip_joint=right_tip,
            pad_z_range=tool.pads.z_range,
            pad_half_width=tool.pads.half_width,
            half_gap_by_fraction=tool.pads.half_gap_by_fraction,
        )
    return CoppeliaSimGripperAdapter(
        RemoteAPIClient(port=port).require("sim"),
        gripper_joints_from_urdf(tool.urdf_path, tool.driven_joint),
        grasp=grasp,
        graspable_bodies=scene.graspable_bodies() if grasp else None,
    )


def _robotiq_modbus_flange(tool: ToolModel, robot: RobotConnectorPort) -> GripperPort:
    """Robotiq 2F por el RS-485 de la brida del CR5: habla por la conexión
    del brazo (el 29999 admite un solo cliente)."""
    from robot_node.adapters.robotiq_2f_adapter import Robotiq2FGripperAdapter

    return Robotiq2FGripperAdapter(robot.command_socket)


REAL_ROBOTS: Dict[str, AdapterEntry] = {
    "cr5_tcp": AdapterEntry(
        _cr5_tcp,
        lambda model: frozenset({caps.ARM_JOINTS, caps.ARM_CARTESIAN}),
        "CR5 por TCP/IP (puertos 29999 y 30004).",
    ),
}
SIM_GRIPPERS: Dict[str, AdapterEntry] = {
    "coppeliasim_urdf_gripper": AdapterEntry(
        _coppeliasim_urdf_gripper,
        lambda tool: frozenset({caps.GRIPPER_ACTUATE} | ({caps.GRIPPER_GRASP_DETECTION} if tool.pads else set())),
        "Pinza de URDF en CoppeliaSim, cinemática; agarre cinemático si el modelo trae `grasp.pads`.",
    ),
}
REAL_GRIPPERS: Dict[str, AdapterEntry] = {
    "robotiq_modbus_flange": AdapterEntry(
        _robotiq_modbus_flange,
        lambda tool: frozenset({caps.GRIPPER_ACTUATE, caps.GRIPPER_GRASP_DETECTION}),
        "Robotiq 2F por Modbus a través del 485 de la brida del CR5 (detecta el agarre por firmware, gOBJ).",
    ),
}


def cell_capabilities(cell) -> FrozenSet[str]:
    """Capacidades de una `CellDescription`: las del robot (simulado o
    real), las de su herramienta y las que salen de combinarlas."""
    robot = cell.robot.model
    if cell.robot.target == "sim":
        result = set(caps.SIM_ROBOT)
    else:
        result = set(REAL_ROBOTS[robot.real_adapter].provides(robot))
    tool = cell.tool
    if tool is not None:
        adapter = tool.model.sim_adapter if cell.robot.target == "sim" else tool.model.real_adapter
        registry = SIM_GRIPPERS if cell.robot.target == "sim" else REAL_GRIPPERS
        if adapter:
            result |= registry[adapter].provides(tool.model)
    return caps.derive(result)


def tool_mount(tool: ToolModel, robot: RobotModel):
    """El `ToolMount` de `coppeliasim_scene_builder` para montar `tool` en
    `robot`."""
    from ..coppeliasim_scene_builder import ToolMount

    mount = tool.mounts[robot.name]
    return ToolMount(
        urdf_path=tool.urdf_path,
        urdf_package_prefix=tool.package_prefix,
        parent_joint=mount.parent_joint,
        offset_pose=mount.offset_pose,
    )
