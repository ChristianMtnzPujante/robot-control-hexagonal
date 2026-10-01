"""Tests de CoppeliaSimGripperAdapter contra un `sim` de mentira y del lector
de mimics sobre el URDF real de assets/robotiq_2f_85/.

NO sustituyen verlo en CoppeliaSim: validan la cuenta (qué ángulo va a
cada joint), no que la malla quede donde debe.
"""

import math
from pathlib import Path

import pytest

from robot_node.adapters.coppeliasim_gripper_adapter import (
    CoppeliaSimGripperAdapter,
    GraspGeometry,
    GripperJoints,
    gripper_joints_from_urdf,
    half_extent_along,
)
from shared_kernel import Body, Box, Cylinder, Pose, Sphere

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


# --- Agarre cinemático -------------------------------------------------------

# Pinza de juguete: nudillos en x = ±0.03, yemas entre 4 y 11 cm por debajo
# de ellos, 4 cm de semiapertura abierta y 0 cerrada, lineal.
_GRASP = GraspGeometry(
    left_knuckle_joint="lk",
    right_knuckle_joint="rk",
    left_tip_joint="lt",
    right_tip_joint="rt",
    pad_z_range=(0.04, 0.11),
    pad_half_width=0.015,
    half_gap_by_fraction=(0.04, 0.02, 0.0),
)
_IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]


class GraspSim(FakeSim):
    """FakeSim con posiciones, matrices y padres. El mundo coincide con el
    marco de agarre: nudillos en z = 0, puntas hacia +z."""

    def __init__(self, bodies):
        super().__init__()
        self.world_positions = {
            "/lk": (0.03, 0.0, 0.0),
            "/rk": (-0.03, 0.0, 0.0),
            "/lt": (0.04, 0.0, 0.05),
            "/rt": (-0.04, 0.0, 0.05),
        }
        self.matrices = {}
        self.parents = {}
        for name, (position, matrix) in bodies.items():
            self.matrices[f"/{name}"] = list(matrix[:3]) + [position[0]] + list(
                matrix[4:7]
            ) + [position[1]] + list(matrix[8:11]) + [position[2]]
            self.parents[f"/{name}"] = "/cuerpos_escena"

    def getObjectPosition(self, handle, relative_to):
        return list(self.world_positions[handle])

    def getObjectMatrix(self, handle, relative_to):
        return self.matrices[handle]

    def getObjectParent(self, handle):
        return self.parents[handle]

    def setObjectParent(self, handle, parent, keep_in_place):
        self.parents[handle] = parent


_CUBE = Body(Box(0.04, 0.04, 0.04), Pose(0, 0, 0), graspable=True)


def _grasping_adapter(position, matrix=_IDENTITY, body=_CUBE):
    sim = GraspSim({"cubo": (position, matrix)})
    adapter = CoppeliaSimGripperAdapter(
        sim, _JOINTS, step_pause_seconds=0.0, grasp=_GRASP, graspable_bodies={"cubo": body}
    )
    return adapter, sim


def test_fraction_at_half_gap_interpolates_the_table():
    assert _GRASP.fraction_at_half_gap(0.04) == 0.0
    assert _GRASP.fraction_at_half_gap(0.03) == pytest.approx(0.25)
    assert _GRASP.fraction_at_half_gap(0.01) == pytest.approx(0.75)
    assert _GRASP.fraction_at_half_gap(0.05) == 0.0
    assert _GRASP.fraction_at_half_gap(-0.01) == 1.0


def test_half_extent_along_each_shape():
    diagonal = (math.sqrt(0.5), math.sqrt(0.5), 0.0)
    assert half_extent_along(Box(0.04, 0.04, 0.1), (1, 0, 0)) == pytest.approx(0.02)
    assert half_extent_along(Box(0.04, 0.04, 0.1), diagonal) == pytest.approx(0.02 * math.sqrt(2))
    assert half_extent_along(Cylinder(0.02, 0.1), (1, 0, 0)) == pytest.approx(0.02)
    assert half_extent_along(Cylinder(0.02, 0.1), (0, 0, 1)) == pytest.approx(0.05)
    assert half_extent_along(Sphere(0.03), diagonal) == pytest.approx(0.03)


def test_closing_on_a_body_between_the_pads_stops_at_contact_and_holds_it():
    adapter, sim = _grasping_adapter((0.0, 0.0, 0.07))
    adapter.set_opening(1.0)
    state = adapter.get_state()
    # Cara del cubo a 0.02 del plano medio -> apertura 0.5 en la tabla.
    assert state.opening == pytest.approx(0.5)
    assert state.holding_object
    assert adapter.held_body == "cubo"
    assert sim.parents["/cubo"] == "/lt"


def test_closing_again_while_holding_does_not_squeeze_further():
    adapter, _ = _grasping_adapter((0.0, 0.0, 0.07))
    adapter.set_opening(1.0)
    adapter.set_opening(1.0)
    assert adapter.get_state().opening == pytest.approx(0.5)


def test_opening_releases_the_body_back_to_its_parent():
    adapter, sim = _grasping_adapter((0.0, 0.0, 0.07))
    adapter.set_opening(1.0)
    adapter.set_opening(0.0)
    assert not adapter.get_state().holding_object
    assert adapter.held_body is None
    assert sim.parents["/cubo"] == "/cuerpos_escena"


def test_an_off_center_body_is_touched_first_by_the_nearer_pad():
    adapter, _ = _grasping_adapter((0.01, 0.0, 0.07))
    adapter.set_opening(1.0)
    # 0.01 de desvío + 0.02 de media cara = 0.03 -> apertura 0.25.
    assert adapter.get_state().opening == pytest.approx(0.25)


def test_a_rotated_box_is_wider_between_the_pads():
    c = math.sqrt(0.5)
    yaw_45 = [c, -c, 0, 0, c, c, 0, 0, 0, 0, 1, 0]
    adapter, _ = _grasping_adapter((0.0, 0.0, 0.07), matrix=yaw_45)
    adapter.set_opening(1.0)
    expected = _GRASP.fraction_at_half_gap(0.02 * math.sqrt(2))
    assert adapter.get_state().opening == pytest.approx(expected)


@pytest.mark.parametrize(
    "position",
    [(0.0, 0.0, 0.2), (0.0, 0.0, 0.01), (0.0, 0.05, 0.07)],
    ids=["demasiado lejos", "demasiado cerca", "fuera de las yemas en y"],
)
def test_a_body_outside_the_pads_is_not_grasped(position):
    adapter, sim = _grasping_adapter(position)
    adapter.set_opening(1.0)
    assert adapter.get_state().opening == pytest.approx(1.0)
    assert not adapter.get_state().holding_object
    assert sim.parents["/cubo"] == "/cuerpos_escena"


def test_a_body_wider_than_the_open_gripper_is_not_grasped():
    wide = Body(Box(0.1, 0.1, 0.02), Pose(0, 0, 0), graspable=True)
    adapter, _ = _grasping_adapter((0.0, 0.0, 0.07), body=wide)
    adapter.set_opening(1.0)
    assert not adapter.get_state().holding_object


def test_without_grasp_geometry_it_never_holds():
    adapter, _ = _adapter()
    adapter.set_opening(1.0)
    assert not adapter.get_state().holding_object
