"""Tests de capacidades (qué puede hacer una célula por cómo está
montada) y de operaciones disponibles (qué se puede hacer ahora)."""

from commander.cell import capabilities as caps
from commander.cell.adapters import cell_capabilities

from cell_fixtures import PADS, TOOL, cell


def _names(operations):
    return {op.name: dict(op.options) for op in operations}


def test_a_simulated_cell_with_grasp_geometry_can_pick_and_place(tree):
    with_pads = dict(TOOL, grasp={"offset": 0.1, "pads": PADS})
    capabilities = cell_capabilities(tree(tool=with_pads))
    assert capabilities == {
        caps.ARM_JOINTS, caps.ARM_CARTESIAN, caps.SIM_GROUND_TRUTH, caps.SIM_RESET,
        caps.GRIPPER_ACTUATE, caps.GRIPPER_GRASP_DETECTION, caps.PICK_AND_PLACE,
    }


def test_a_simulated_gripper_without_pads_cannot_detect_grasps(tree):
    capabilities = cell_capabilities(tree())
    assert caps.GRIPPER_ACTUATE in capabilities
    assert caps.GRIPPER_GRASP_DETECTION not in capabilities and caps.PICK_AND_PLACE not in capabilities


def test_the_real_robot_never_offers_simulator_capabilities(tree):
    real = cell(robot={"ref": "robots/juguete.yaml", "target": "real", "host": "10.0.0.1"},
                tools=[{"ref": "tools/pinza.yaml", "grasp_offset": 0.12}])
    capabilities = cell_capabilities(tree(cell_data=real))
    assert caps.SIM_GROUND_TRUTH not in capabilities and caps.SIM_RESET not in capabilities
    assert caps.PICK_AND_PLACE in capabilities  # la Robotiq detecta el agarre por firmware


def test_an_arm_without_tool_only_moves(tree):
    capabilities = cell_capabilities(tree(cell_data={"name": "brazo", "robot": {"ref": "robots/juguete.yaml"}}))
    assert capabilities == {caps.ARM_JOINTS, caps.ARM_CARTESIAN, caps.SIM_GROUND_TRUTH, caps.SIM_RESET}


_FULL = frozenset({caps.ARM_JOINTS, caps.ARM_CARTESIAN, caps.GRIPPER_ACTUATE,
                   caps.GRIPPER_GRASP_DETECTION, caps.PICK_AND_PLACE})


def test_with_empty_hands_you_can_pick_but_not_place():
    ops = _names(caps.available_operations(_FULL, ["home"], ["destino"], ["cubo", "lata"], holding=False))
    assert ops["pick"] == {"body": ("cubo", "lata")}
    assert "place" not in ops and "close_gripper" in ops
    assert ops["move_to_posture"] == {"posture": ("home",)}


def test_while_holding_you_can_place_but_not_pick_or_close():
    ops = _names(caps.available_operations(_FULL, ["home"], ["destino"], ["cubo", "lata"],
                                           holding=True, held_body="cubo"))
    assert ops["place"] == {"point": ("destino",)}
    assert "pick" not in ops and "close_gripper" not in ops and "open_gripper" in ops


def test_without_knowing_the_gripper_state_neither_pick_nor_place_is_offered():
    ops = _names(caps.available_operations(_FULL, [], ["destino"], ["cubo"], holding=None))
    assert "pick" not in ops and "place" not in ops


def test_nothing_to_pick_means_no_pick():
    ops = _names(caps.available_operations(_FULL, [], ["destino"], [], holding=False))
    assert "pick" not in ops


def test_simulator_operations_come_from_simulator_capabilities():
    ops = _names(caps.available_operations(frozenset({caps.SIM_GROUND_TRUTH, caps.SIM_RESET}), [], [], [], None))
    assert set(ops) == {"refresh_world", "reset_cell"}
