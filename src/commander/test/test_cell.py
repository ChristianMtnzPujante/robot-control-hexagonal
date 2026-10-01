"""Tests de los formatos de `descriptions/` y de la compilación de una
célula. No construyen nada (eso es `open_direct`, que necesita CoppeliaSim
o el robot): validan que cada pieza se lee bien, que los errores dicen
dónde están, que las reglas entre piezas se cumplen y que la guía está al
día con los esquemas.

La mayoría usa un árbol `descriptions/` de juguete en un directorio
temporal (robot con `joint_names` explícitos, URDF vacío); solo los de la
célula real del repo necesitan el URDF del CR5."""

import math
import re
from pathlib import Path

import pytest
import yaml

from commander.cell import CellHandle, InvalidCellError, compile_cell, load_robot, load_tool, parse_cell, resolve_cell
from commander.cell.elements import parse_robot, parse_tool
from commander.cell.guide import GUIDE_PATH, render
from commander.cell.scene_format import parse_scene, rpy_degrees_to_quaternion
from shared_kernel import Box, Cylinder, Point

from cell_fixtures import ROBOT as _ROBOT, SCENE as _SCENE, TOOL as _TOOL, cell as _cell

_CR5_URDF = Path("~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/cr5_robot.urdf").expanduser()
needs_cr5_urdf = pytest.mark.skipif(not _CR5_URDF.exists(), reason="sin el URDF del CR5 en esta máquina")

# --- La célula real del repo -----------------------------------------------------


@needs_cr5_urdf
def test_mesa_cubo_compiles_from_its_four_pieces():
    cell = compile_cell("mesa_cubo")
    assert cell.robot.model.name == "cr5" and cell.robot.target == "sim"
    assert cell.robot.model.joint_names == ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    assert cell.tool.model.name == "robotiq_2f_85"
    assert cell.tool.effective_grasp_offset == pytest.approx(0.14)
    assert cell.tool.model.mounts["cr5"].parent_joint == "joint6"
    assert len(cell.tool.model.pads.half_gap_by_fraction) == 9
    assert set(cell.postures) == {"home", "pre_agarre"}  # home del robot, pre_agarre de la célula
    assert set(cell.scene.graspable_bodies()) == {"cubo", "lata"}
    assert cell.scene.bodies["lata"].shape == Cylinder(0.025, 0.10)
    assert cell.scene.objects["destino"] == Point(-0.528, 0.259, 0.025)


@needs_cr5_urdf
def test_mesa_cubo_cannot_go_to_the_real_robot_without_a_measured_grasp_offset():
    with pytest.raises(InvalidCellError, match="grasp_offset MEDIDO"):
        compile_cell("mesa_cubo").with_target("real", "192.168.5.1")


def test_resolve_cell_accepts_a_name_or_a_path():
    by_name = resolve_cell("mesa_cubo")
    assert resolve_cell(str(by_name)) == by_name


def test_an_unknown_cell_lists_the_available_ones():
    with pytest.raises(InvalidCellError, match="mesa_cubo"):
        resolve_cell("no_existe")


def test_robot_and_tool_load_by_name():
    assert load_tool("robotiq_2f_85").driven_joint == "robotiq_85_left_knuckle_joint"
    if _CR5_URDF.exists():
        assert load_robot("cr5").real_adapter == "cr5_tcp"


# --- Composición -----------------------------------------------------------------


def test_a_cell_merges_robot_and_task_postures(tree):
    cell = tree()
    assert cell.postures == {"home": (0.0, 0.0, 0.0), "trabajo": (10.0, 20.0, 30.0)}


def test_a_task_posture_cannot_reuse_a_robot_posture_name(tree):
    with pytest.raises(InvalidCellError, match='"home" ya la define el robot'):
        tree(cell_data=_cell(postures={"home": [1, 2, 3]}))


def test_a_posture_needs_one_value_per_joint(tree):
    with pytest.raises(InvalidCellError, match="2 valores y el robot 3 joints"):
        tree(cell_data=_cell(postures={"corta": [1, 2]}))


def test_the_tool_needs_a_mount_for_this_robot(tree):
    other = dict(_TOOL, mounts={"otro_robot": {"parent_joint": "j3"}})
    with pytest.raises(InvalidCellError, match='no tiene montaje previsto en el robot "juguete"'):
        tree(tool=other)


def test_mount_offsets_become_a_pose(tree):
    mount = tree().tool.model.mounts["juguete"]
    assert mount.offset_pose[:3] == (0.0, 0.0, 0.02)
    assert mount.offset_pose[3:] == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))


def test_a_cell_without_tools_or_scene_is_valid(tree):
    cell = tree(cell_data={"name": "solo_brazo", "robot": {"ref": "robots/juguete.yaml"}})
    assert cell.tool is None and cell.scene.bodies == {}


def test_real_needs_host_measured_offset_and_real_sections(tree):
    real = {"ref": "robots/juguete.yaml", "target": "real"}
    with pytest.raises(InvalidCellError, match="host"):
        tree(cell_data=_cell(robot=real))
    with pytest.raises(InvalidCellError, match="grasp_offset MEDIDO"):
        tree(cell_data=_cell(robot=dict(real, host="10.0.0.1")))
    measured = _cell(robot=dict(real, host="10.0.0.1"), tools=[{"ref": "tools/pinza.yaml", "grasp_offset": 0.15}])
    assert tree(cell_data=measured).tool.effective_grasp_offset == 0.15
    sim_only = {k: v for k, v in _ROBOT.items() if k != "real"}
    with pytest.raises(InvalidCellError, match='no tiene sección "real"'):
        tree(cell_data=measured, robot=sim_only)


def test_unknown_adapters_are_caught_when_compiling(tree):
    with pytest.raises(InvalidCellError, match='adaptador "pinza_magica" desconocido'):
        tree(tool=dict(_TOOL, sim={"adapter": "pinza_magica"}))


@pytest.mark.parametrize(
    "cell, message",
    [
        (_cell(kinematics="dh"), 'kinematics "dh"'),
        (_cell(tools=[{"ref": "tools/pinza.yaml"}, {"ref": "tools/pinza.yaml"}]), "una sola herramienta"),
        (_cell(robot={"ref": "robots/juguete.yaml", "initial_posture": "nada"}), "initial_posture"),
        (_cell(robot={"ref": "robots/juguete.yaml", "target": "luna"}), 'target "luna"'),
        (_cell(escena={}), "célula: clave(s) desconocida(s) escena"),
        ({"name": "x"}, 'célula: falta "robot"'),
        (_cell(robot={"ref": "robots/no_existe.yaml"}), 'no encuentro "robots/no_existe.yaml"'),
    ],
    ids=["cinemática", "dos herramientas", "postura inicial", "destino", "clave desconocida", "sin robot", "ref rota"],
)
def test_cell_errors_say_what_is_wrong(tree, cell, message):
    with pytest.raises(InvalidCellError, match=re.escape(message)):
        tree(cell_data=cell)


def test_compile_cell_prefixes_errors_with_the_file(tmp_path):
    path = tmp_path / "rota.yaml"
    path.write_text("name: rota\n")
    with pytest.raises(InvalidCellError, match="rota.yaml.*robot"):
        compile_cell(path)


# --- Robot y herramienta ----------------------------------------------------------


def test_robot_errors(tmp_path):
    (tmp_path / "robot.urdf").write_text("<robot/>")
    with pytest.raises(InvalidCellError, match=re.escape("robots/r.urdf: no existe")):
        parse_robot(dict(_ROBOT, urdf="falta.urdf"), "r", tmp_path)
    with pytest.raises(InvalidCellError, match="robots/r.postures.home debería tener 3"):
        parse_robot(dict(_ROBOT, postures={"home": [0, 0]}), "r", tmp_path)
    with pytest.raises(InvalidCellError, match="no pude sacar los joints del URDF"):
        parse_robot({k: v for k, v in _ROBOT.items() if k != "joint_names"}, "r", tmp_path)


def test_tool_pad_errors(tmp_path):
    (tmp_path / "tool.urdf").write_text("<robot/>")
    pads = {"frame_joints": {"left_knuckle": "a", "right_knuckle": "b", "left_tip": "c", "right_tip": "d"},
            "z_range": [0.04, 0.11], "half_width": 0.01, "half_gap_by_fraction": [0.04, 0.02, 0.0]}
    tool = parse_tool(dict(_TOOL, grasp={"offset": 0.1, "pads": pads}), "t", tmp_path)
    assert tool.pads.frame_joints == ("a", "b", "c", "d")
    incomplete = dict(pads, frame_joints={"left_knuckle": "a"})
    with pytest.raises(InvalidCellError, match="frame_joints necesita exactamente"):
        parse_tool(dict(_TOOL, grasp={"offset": 0.1, "pads": incomplete}), "t", tmp_path)
    growing = dict(pads, half_gap_by_fraction=[0.0, 0.04])
    with pytest.raises(InvalidCellError, match="decrecientes"):
        parse_tool(dict(_TOOL, grasp={"offset": 0.1, "pads": growing}), "t", tmp_path)


# --- Escena ----------------------------------------------------------------------


def _body(**fields):
    return parse_scene({"bodies": {"cubo": fields}}, "scenes/s")


def test_rpy_degrees_follow_the_urdf_convention():
    assert rpy_degrees_to_quaternion(0, 0, 90) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert rpy_degrees_to_quaternion(180, 0, 0) == pytest.approx((1, 0, 0, 0), abs=1e-12)


def test_scene_reads_every_section():
    scene = parse_scene(
        dict(_SCENE, obstacles={"poste": {"center": [0.3, 0, 0.4], "radius": 0.05}},
             planes={"suelo": {"point": [0, 0, 0], "normal": [0, 0, 1]}}),
        "scenes/s",
    )
    assert scene.bodies["cubo"].shape == Box(0.05, 0.05, 0.05) and scene.bodies["cubo"].graspable
    assert scene.objects["destino"] == Point(0.3, 0.2, 0.025)
    assert scene.obstacles["poste"].radius == 0.05
    assert scene.planes["suelo"].normal == Point(0, 0, 1)


@pytest.mark.parametrize(
    "fields, message",
    [
        (dict(shape="box", size=[0.05] * 3, position=[0, 0, 0], graspabel=True), "scenes/s.bodies.cubo: clave(s) desconocida(s) graspabel"),
        (dict(shape="cono", position=[0, 0, 0]), 'scenes/s.bodies.cubo.shape "cono"'),
        (dict(shape="box", size=[0.05, 0.05], position=[0, 0, 0]), "scenes/s.bodies.cubo.size debería tener 3"),
        (dict(shape="box", size=[0.05] * 3), 'scenes/s.bodies.cubo: falta "position"'),
        (dict(shape="box", size=[0.05, -1, 0.05], position=[0, 0, 0]), "scenes/s.bodies.cubo: size_y"),
        (dict(shape="box", size=[0.05] * 3, radius=0.1, position=[0, 0, 0]), "radius no aplica a shape: box"),
        (dict(shape="sphere", radius="grande", position=[0, 0, 0]), "scenes/s.bodies.cubo.radius debería ser un número"),
        (dict(shape="sphere", radius=0.02, position=[0, 0, 0], graspable="si"), "graspable debería ser true o false"),
        (dict(shape="cylinder", radius=0.02, position=[0, 0, 0]), 'scenes/s.bodies.cubo: falta "height"'),
        (dict(shape="sphere", radius=0.02, position=[0, 0, 0], rpy_degrees=[0, 0, 0], quaternion=[0, 0, 0, 1]), "no los dos"),
    ],
    ids=["clave mal escrita", "forma desconocida", "medidas incompletas", "sin posición", "medida negativa",
         "campo de otra forma", "no numérico", "booleano mal escrito", "cilindro sin altura", "dos orientaciones"],
)
def test_body_errors_say_where_they_are(fields, message):
    with pytest.raises(InvalidCellError, match=re.escape(message)):
        _body(**fields)


# --- Posturas y guía -------------------------------------------------------------


def test_postures_become_joint_configurations_in_radians(tree):
    handle = CellHandle(tree(), manipulator=None)
    assert handle.posture("trabajo").angle_of("j2") == pytest.approx(math.radians(20))
    with pytest.raises(InvalidCellError, match="Definidas: home, trabajo"):
        handle.posture("saludo")


def test_the_guide_in_the_vault_is_up_to_date():
    assert GUIDE_PATH.read_text() == render(), (
        "La guía de formatos no coincide con los esquemas: regenérala con "
        "`ros2 run commander cell_guide --write`."
    )
