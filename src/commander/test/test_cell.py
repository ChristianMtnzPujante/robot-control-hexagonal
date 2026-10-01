"""Tests de la descripción de célula, su lector de YAML y el catálogo. No
construyen nada (eso es `open_direct`, que necesita CoppeliaSim o el
robot): validan que lo escrito en YAML llega bien al dominio y que los
errores dicen dónde están."""

import math
import re

import pytest
import yaml

from commander.cell import CellHandle, InvalidCellError, load_cell, parse_cell, resolve_scenario
from commander.cell.catalog import robot_model, tool_model
from commander.cell.description import CellDescription, RobotSpec, ToolSpec
from commander.cell.loader import rpy_degrees_to_quaternion
from shared_kernel import Box, Cylinder, Point

_MINIMAL = """
name: prueba
robot: {model: cr5}
"""


def _parse(text):
    return parse_cell(yaml.safe_load(text))


def _with_body(body_yaml):
    return _parse(_MINIMAL + "scene:\n  bodies:\n    cubo:\n" + body_yaml)


# --- El escenario del repo -------------------------------------------------------


def test_mesa_cubo_loads_by_name_with_everything_in_place():
    cell = load_cell("mesa_cubo")
    assert cell.name == "mesa_cubo"
    assert cell.robot == RobotSpec(model="cr5", target="sim", initial_posture="home")
    assert cell.tools == (ToolSpec(model="robotiq_2f_85"),)
    assert cell.postures["pre_agarre"] == (0.0, 20.0, 100.0, -30.0, -90.0, 0.0)
    assert set(cell.scene.bodies) == {"mesa", "cubo", "lata"}
    assert set(cell.scene.graspable_bodies()) == {"cubo", "lata"}
    assert cell.scene.bodies["cubo"].shape == Box(0.05, 0.05, 0.05)
    assert cell.scene.bodies["lata"].shape == Cylinder(0.025, 0.10)
    assert cell.scene.bodies["lata"].color == (0.15, 0.35, 0.85)
    assert cell.scene.objects["destino"] == Point(-0.528, 0.259, 0.025)


def test_resolve_scenario_accepts_name_name_with_suffix_and_path():
    by_name = resolve_scenario("mesa_cubo")
    assert resolve_scenario("mesa_cubo.yaml") == by_name
    assert resolve_scenario(str(by_name)) == by_name


def test_an_unknown_scenario_lists_the_available_ones():
    with pytest.raises(InvalidCellError, match="mesa_cubo"):
        resolve_scenario("no_existe")


def test_mesa_cubo_cannot_go_to_the_real_robot_without_a_measured_grasp_offset():
    with pytest.raises(InvalidCellError, match="grasp_offset"):
        load_cell("mesa_cubo").with_target("real", "192.168.5.1")


# --- Lector: formas, poses y errores con su sitio ----------------------------------


def test_defaults_of_a_minimal_cell():
    cell = _parse(_MINIMAL)
    assert cell.robot.target == "sim" and cell.kinematics == "poe"
    assert cell.tools == () and cell.scene.bodies == {}
    assert cell.simulator.port == 23000


def test_rpy_degrees_follow_the_urdf_convention():
    assert rpy_degrees_to_quaternion(0, 0, 90) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert rpy_degrees_to_quaternion(180, 0, 0) == pytest.approx((1, 0, 0, 0), abs=1e-12)


def test_a_body_can_be_oriented_with_rpy_or_quaternion_but_not_both():
    rotated = _with_body("      shape: sphere\n      radius: 0.02\n      position: [0, 0, 0]\n      rpy_degrees: [0, 0, 90]\n")
    assert rotated.scene.bodies["cubo"].pose.qz == pytest.approx(math.sqrt(0.5))
    with pytest.raises(InvalidCellError, match="no los dos"):
        _with_body(
            "      shape: sphere\n      radius: 0.02\n      position: [0, 0, 0]\n"
            "      rpy_degrees: [0, 0, 0]\n      quaternion: [0, 0, 0, 1]\n"
        )


def test_obstacles_and_planes_reach_the_scene():
    cell = _parse(
        _MINIMAL
        + "scene:\n  obstacles:\n    poste: {center: [0.3, 0, 0.4], radius: 0.05}\n"
        + "  planes:\n    suelo: {point: [0, 0, 0], normal: [0, 0, 1]}\n"
    )
    assert cell.scene.obstacles["poste"].radius == 0.05
    assert cell.scene.planes["suelo"].normal == Point(0, 0, 1)


@pytest.mark.parametrize(
    "body_yaml, where",
    [
        ("      shape: box\n      size: [0.05, 0.05, 0.05]\n      position: [0, 0, 0]\n      graspabel: true\n",
         "scene.bodies.cubo: clave(s) desconocida(s) graspabel"),
        ("      shape: cono\n      position: [0, 0, 0]\n", 'scene.bodies.cubo.shape "cono"'),
        ("      shape: box\n      size: [0.05, 0.05]\n      position: [0, 0, 0]\n", "scene.bodies.cubo.size debería tener 3"),
        ("      shape: box\n      size: [0.05, 0.05, 0.05]\n", 'scene.bodies.cubo: falta "position"'),
        ("      shape: box\n      size: [0.05, -1, 0.05]\n      position: [0, 0, 0]\n", "scene.bodies.cubo: size_y"),
        ("      shape: sphere\n      radius: grande\n      position: [0, 0, 0]\n", "scene.bodies.cubo.radius debería ser un número"),
        ("      shape: sphere\n      radius: 0.02\n      position: [0, 0, 0]\n      graspable: si\n", "graspable debería ser true o false"),
        ("      shape: cylinder\n      radius: 0.02\n      position: [0, 0, 0]\n", 'scene.bodies.cubo: falta "height"'),
    ],
    ids=["clave mal escrita", "forma desconocida", "medidas incompletas", "sin posición",
         "medida negativa", "no numérico", "booleano mal escrito", "cilindro sin altura"],
)
def test_body_errors_say_where_they_are(body_yaml, where):
    with pytest.raises(InvalidCellError, match=re.escape(where)):
        _with_body(body_yaml)


def test_unknown_top_level_key_is_an_error():
    with pytest.raises(InvalidCellError, match="la raíz: clave"):
        _parse(_MINIMAL + "escena: {}\n")


def test_a_missing_robot_is_an_error():
    with pytest.raises(InvalidCellError, match='falta "robot"'):
        _parse("name: prueba\n")


def test_load_cell_prefixes_errors_with_the_file(tmp_path):
    path = tmp_path / "rota.yaml"
    path.write_text("name: rota\nrobot: {model: cr5, target: luna}\n")
    with pytest.raises(InvalidCellError, match="rota.yaml.*luna"):
        load_cell(path)


# --- Reglas de la descripción ------------------------------------------------------


def test_real_needs_a_host():
    with pytest.raises(InvalidCellError, match="host"):
        CellDescription(name="c", robot=RobotSpec(model="cr5", target="real"))


def test_real_with_host_and_measured_offset_is_valid():
    cell = CellDescription(
        name="c",
        robot=RobotSpec(model="cr5", target="real", host="192.168.5.1"),
        tools=(ToolSpec(model="robotiq_2f_85", grasp_offset=0.15),),
    )
    assert cell.tool.grasp_offset == 0.15


def test_an_undefined_initial_posture_is_an_error():
    with pytest.raises(InvalidCellError, match="initial_posture"):
        CellDescription(name="c", robot=RobotSpec(model="cr5", initial_posture="nada"))


def test_only_one_tool_for_now():
    with pytest.raises(InvalidCellError, match="una sola herramienta"):
        CellDescription(name="c", robot=RobotSpec(model="cr5"),
                        tools=(ToolSpec("robotiq_2f_85"), ToolSpec("robotiq_2f_85")))


def test_unknown_kinematics_is_an_error():
    with pytest.raises(InvalidCellError, match="kinematics"):
        CellDescription(name="c", robot=RobotSpec(model="cr5"), kinematics="dh")


# --- Catálogo y posturas -----------------------------------------------------------


def test_catalog_knows_the_cr5_and_the_2f_85_on_it():
    assert robot_model("cr5").joint_names == ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    assert tool_model("robotiq_2f_85", "cr5").sim_grasp_offset == pytest.approx(0.14)


def test_catalog_errors_list_what_exists():
    with pytest.raises(InvalidCellError, match="cr5"):
        robot_model("ur5")
    with pytest.raises(InvalidCellError, match="robotiq_2f_85"):
        tool_model("pinza_magica", "cr5")
    with pytest.raises(InvalidCellError, match="montaje"):
        tool_model("robotiq_2f_85", "panda")


def test_postures_become_joint_configurations_in_radians():
    handle = CellHandle(load_cell("mesa_cubo"), manipulator=None, joint_names=robot_model("cr5").joint_names)
    posture = handle.posture("pre_agarre")
    assert posture.angle_of("joint3") == pytest.approx(math.radians(100))
    with pytest.raises(InvalidCellError, match="Definidas: home, pre_agarre"):
        handle.posture("saludo")


def test_a_posture_with_the_wrong_number_of_joints_is_an_error():
    cell = _parse(_MINIMAL + "postures:\n  corta: [0, 0, 0]\n")
    handle = CellHandle(cell, manipulator=None, joint_names=robot_model("cr5").joint_names)
    with pytest.raises(InvalidCellError, match="3 valores y el robot 6"):
        handle.posture("corta")
