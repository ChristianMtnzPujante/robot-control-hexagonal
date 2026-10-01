"""Catálogo de modelos que una `CellDescription` puede nombrar: qué es
"cr5" o "robotiq_2f_85" en concreto (su URDF, cómo se monta en la escena,
cómo se crea su adaptador en simulación y en el robot real). Añadir un
robot o una herramienta nueva es añadir una entrada aquí, no tocar los
scripts.

Los adaptadores se importan dentro de las factorías (no arriba): así el
catálogo se puede leer -- y la descripción validar -- sin tener CoppeliaSim
ni el robot a mano.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Tuple

from shared_kernel import GripperPort, Scene

from ..coppeliasim_scene_builder import (
    CR5_JOINT_NAMES,
    CR5_TIP_NAME,
    CR5_URDF_PACKAGE_PREFIX,
    CR5_URDF_PATH,
    ROBOTIQ_2F_85_GRASP_DEPTH,
    ROBOTIQ_2F_85_ON_CR5,
    ToolMount,
    robotiq_2f_85_gripper,
)
from .description import InvalidCellError


@dataclass(frozen=True)
class RobotModel:
    """`root_link_visual_alias` es el objeto raíz que simURDF crea al
    importar (para poder borrarlo al reconstruir la escena); `sim_tip_name`
    el objeto de CoppeliaSim que se usa de tip para el rastro visual."""

    urdf_path: str
    urdf_package_prefix: str
    base_link: str
    tip_link: str
    joint_names: Tuple[str, ...]
    sim_tip_name: str
    root_link_visual_alias: str


@dataclass(frozen=True)
class ToolModel:
    """`mounts` dice dónde se cuelga en CADA robot del catálogo (una
    herramienta puede no estar prevista para un robot). `sim_grasp_offset`
    es la distancia de agarre del modelo, solo válida en simulación.
    `build_sim` recibe (puerto, escena) y `build_real` el adaptador real
    del robot (la Robotiq habla por su conexión)."""

    mounts: Dict[str, ToolMount]
    sim_grasp_offset: float
    build_sim: Callable[[int, Scene], GripperPort]
    build_real: Callable[[object], GripperPort]


def _robotiq_2f_85_real(robot) -> GripperPort:
    from robot_node.adapters.robotiq_2f_adapter import Robotiq2FGripperAdapter

    return Robotiq2FGripperAdapter(robot.command_socket)


ROBOTS: Dict[str, RobotModel] = {
    "cr5": RobotModel(
        urdf_path=CR5_URDF_PATH,
        urdf_package_prefix=CR5_URDF_PACKAGE_PREFIX,
        base_link="base_link",
        tip_link="Link6",
        joint_names=tuple(CR5_JOINT_NAMES),
        sim_tip_name=CR5_TIP_NAME,
        root_link_visual_alias="dummy_link_visual",
    ),
}

TOOLS: Dict[str, ToolModel] = {
    "robotiq_2f_85": ToolModel(
        mounts={"cr5": ROBOTIQ_2F_85_ON_CR5},
        sim_grasp_offset=ROBOTIQ_2F_85_GRASP_DEPTH,
        build_sim=lambda port, scene: robotiq_2f_85_gripper(port, scene=scene),
        build_real=_robotiq_2f_85_real,
    ),
}


def robot_model(name: str) -> RobotModel:
    if name not in ROBOTS:
        raise InvalidCellError(f'robot "{name}" no está en el catálogo: {", ".join(ROBOTS)}')
    return ROBOTS[name]


def tool_model(name: str, robot: str) -> ToolModel:
    if name not in TOOLS:
        raise InvalidCellError(f'herramienta "{name}" no está en el catálogo: {", ".join(TOOLS)}')
    model = TOOLS[name]
    if robot not in model.mounts:
        raise InvalidCellError(f'la herramienta "{name}" no tiene montaje previsto en el robot "{robot}"')
    return model
