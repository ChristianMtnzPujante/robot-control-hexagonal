"""Un árbol `descriptions/` de juguete para los tests: robot de 3 joints
con `joint_names` explícitos (no necesita un URDF real), una pinza montada
en él y una escena con un cubo. El fixture `tree` está en `conftest.py`."""

from pathlib import Path

import yaml

from commander.cell import parse_cell

ROBOT = {
    "urdf": "robot.urdf",
    "package_prefix": ".",
    "base_link": "base",
    "tip_link": "flange",
    "joint_names": ["j1", "j2", "j3"],
    "postures": {"home": [0, 0, 0]},
    "sim": {"root_alias": "base_visual"},
    "real": {"adapter": "cr5_tcp"},
}
TOOL = {
    "urdf": "tool.urdf",
    "package_prefix": ".",
    "driven_joint": "finger",
    "mounts": {"juguete": {"parent_joint": "j3", "position": [0, 0, 0.02], "rpy_degrees": [0, 0, 90]}},
    "grasp": {"offset": 0.1},
    "sim": {"adapter": "coppeliasim_urdf_gripper"},
    "real": {"adapter": "robotiq_modbus_flange"},
}
PADS = {
    "frame_joints": {"left_knuckle": "a", "right_knuckle": "b", "left_tip": "c", "right_tip": "d"},
    "z_range": [0.04, 0.11],
    "half_width": 0.01,
    "half_gap_by_fraction": [0.04, 0.02, 0.0],
}
SCENE = {
    "bodies": {"cubo": {"shape": "box", "size": [0.05, 0.05, 0.05], "position": [0.3, 0, 0.025], "graspable": True}},
    "points": {"destino": [0.3, 0.2, 0.025]},
}


def cell(**overrides):
    data = {
        "name": "prueba",
        "robot": {"ref": "robots/juguete.yaml"},
        "tools": [{"ref": "tools/pinza.yaml"}],
        "scene": {"ref": "scenes/mesa.yaml"},
        "postures": {"trabajo": [10, 20, 30]},
    }
    data.update(overrides)
    return data


def make_tree(root: Path):
    """Devuelve una función que compila una célula (dict) sobre un árbol
    `descriptions/` escrito en `root`, con piezas opcionalmente cambiadas."""

    def write(kind, name, data):
        (root / kind).mkdir(exist_ok=True)
        (root / kind / f"{name}.yaml").write_text(yaml.safe_dump(data))

    for kind in ("robots", "tools"):
        (root / kind).mkdir(exist_ok=True)
        (root / kind / "robot.urdf").write_text("<robot/>")
        (root / kind / "tool.urdf").write_text("<robot/>")

    def compile_with(cell_data=None, robot=None, tool=None, scene=None):
        write("robots", "juguete", robot or ROBOT)
        write("tools", "pinza", tool or TOOL)
        write("scenes", "mesa", scene if scene is not None else SCENE)
        return parse_cell(cell_data or cell(), root)

    return compile_with
