"""Tests de CoppeliaSimGripperAdapter contra un `sim` de mentira y del lector
de mimics sobre el URDF real de assets/robotiq_2f_85/.

NO sustituyen verlo en CoppeliaSim: validan la cuenta (qué ángulo va a
cada joint), no que la malla quede donde debe.
"""

from pathlib import Path

import pytest

from robot_node.adapters.coppeliasim_gripper_adapter import (
    CoppeliaSimGripperAdapter,
    GripperJoints,
    gripper_joints_from_urdf,
)

_URDF = (
    Path(__file__).resolve().parents[3]
    / "assets/robotiq_2f_85/urdf/robotiq_2f_85.urdf"
)


class FakeSim:
    def __init__(self):
        self.positions = {}

    def getObject(self, path):
        self.positions.setdefault(path, 0.0)
        return path

    def getJointPosition(self, handle):
        return self.positions[handle]

    def setJointPosition(self, handle, angle):
        self.positions[handle] = angle


_JOINTS = GripperJoints(
    driven_joint="drive",
    closed_angle=0.8,
    multipliers={"drive": 1.0, "mirror": -1.0, "follower": 1.0},
)


def _adapter():
    sim = FakeSim()
    return CoppeliaSimGripperAdapter(sim, _JOINTS, step_pause_seconds=0.0), sim


def test_closing_moves_every_joint_by_its_multiplier():
    adapter, sim = _adapter()
    adapter.set_opening(1.0)
    assert sim.positions == pytest.approx(
        {"/drive": 0.8, "/mirror": -0.8, "/follower": 0.8}
    )


def test_state_reports_the_opening_it_was_given():
    adapter, _ = _adapter()
    adapter.set_opening(0.5)
    state = adapter.get_state()
    assert state.opening == pytest.approx(0.5)
    assert state.activated and not state.holding_object and state.fault_code == 0


def test_opening_again_returns_to_zero():
    adapter, sim = _adapter()
    adapter.set_opening(1.0)
    adapter.set_opening(0.0)
    assert all(angle == pytest.approx(0.0) for angle in sim.positions.values())


@pytest.mark.parametrize("fraction", [-0.1, 1.1])
def test_rejects_openings_out_of_range(fraction):
    adapter, sim = _adapter()
    with pytest.raises(ValueError):
        adapter.set_opening(fraction)
    assert all(angle == 0.0 for angle in sim.positions.values())


def test_reads_the_robotiq_2f_85_mimics_from_its_urdf():
    joints = gripper_joints_from_urdf(str(_URDF), "robotiq_85_left_knuckle_joint")
    assert joints.closed_angle == pytest.approx(0.8)
    assert joints.multipliers == {
        "robotiq_85_left_knuckle_joint": 1.0,
        "robotiq_85_right_knuckle_joint": -1.0,
        "robotiq_85_left_inner_knuckle_joint": 1.0,
        "robotiq_85_right_inner_knuckle_joint": -1.0,
        "robotiq_85_left_finger_tip_joint": -1.0,
        "robotiq_85_right_finger_tip_joint": 1.0,
    }


def test_unknown_driven_joint_is_an_error():
    with pytest.raises(ValueError):
        gripper_joints_from_urdf(str(_URDF), "no_existe")
