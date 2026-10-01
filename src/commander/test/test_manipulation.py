"""Tests de `Manipulator` contra un robot y una pinza de mentira, pero con
la cinemática PoE real del CR5: validan las cuentas (dónde cierra, que las
rectas son rectas, qué pasa si no coge nada), no el hardware."""

import math

import pytest

from commander.manipulation import (
    GraspFailedError,
    GraspSettings,
    Manipulator,
    OperationCancelledError,
    tool_axis,
    top_down_quaternion,
)
from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import (
    Body,
    Box,
    GripperState,
    JointConfiguration,
    JointPosition,
    Point,
    Pose,
)

_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Brida mirando hacia abajo en (-0.571, -0.141, 0.265).
_WORK_DEGREES = [0.0, 20.0, 100.0, -30.0, -90.0, 0.0]
_SETTINGS = GraspSettings(grasp_offset=0.14, gripper_poll_seconds=0.0, gripper_timeout_seconds=0.1)
_KINEMATICS = PoeKinematicsAdapter()


def _configuration(degrees):
    return JointConfiguration.create(
        [JointPosition(n, math.radians(d)) for n, d in zip(_JOINTS, degrees)]
    ).value


class FakeRobot:
    def __init__(self, configuration):
        self.configuration = configuration
        self.history = [configuration]

    def set_joints(self, configuration):
        self.configuration = configuration
        self.history.append(configuration)

    def get_current_configuration(self):
        return self.configuration

    def close(self):
        pass


class FakeGripper:
    """Coge algo al cerrar solo si `grabs`; apunta la configuración del
    brazo en cada orden."""

    def __init__(self, robot, grabs=True):
        self._robot = robot
        self._grabs = grabs
        self.opening = 0.0
        self.commands = []

    def activate(self):
        pass

    def set_opening(self, fraction):
        self.opening = fraction
        self.commands.append((fraction, self._robot.configuration))

    def get_state(self):
        holding = self._grabs and self.opening > 0.0
        return GripperState(self.opening, True, holding, 0)

    def close(self):
        pass


def _manipulator(grabs=True, confirm=lambda message: True):
    robot = FakeRobot(_configuration(_WORK_DEGREES))
    gripper = FakeGripper(robot, grabs)
    manipulator = Manipulator(
        robot, gripper, _KINEMATICS, _SETTINGS, confirm=confirm, log=lambda message: None
    )
    return manipulator, robot, gripper


def _flange(configuration):
    return _KINEMATICS.forward_kinematics(configuration)


def _cube(x=-0.571, y=-0.141, z=0.025, graspable=True):
    return Body(Box(0.05, 0.05, 0.05), Pose(x, y, z), graspable=graspable)


def test_tool_axis_of_the_identity_is_z():
    assert tool_axis(Pose(0, 0, 0)) == pytest.approx((0.0, 0.0, 1.0))


def test_the_work_posture_looks_down():
    manipulator, _, _ = _manipulator()
    # abs=1e-5: el URDF del CR5 redondea pi/2 a 1.5708 (3.7e-6 de error).
    assert tool_axis(manipulator.flange_pose()) == pytest.approx((0.0, 0.0, -1.0), abs=1e-5)


def test_move_linear_follows_a_cartesian_straight_line():
    manipulator, robot, _ = _manipulator()
    start = manipulator.flange_pose()
    goal = Pose(start.x + 0.05, start.y - 0.03, start.z - 0.10, start.qx, start.qy, start.qz, start.qw)
    manipulator.move_linear(goal)

    a = (start.x, start.y, start.z)
    b = (goal.x, goal.y, goal.z)
    ab = [bi - ai for ai, bi in zip(a, b)]
    for configuration in robot.history[1:]:
        p = _flange(configuration)
        ap = (p.x - a[0], p.y - a[1], p.z - a[2])
        t = sum(u * v for u, v in zip(ap, ab)) / sum(u * u for u in ab)
        closest = [ai + t * d for ai, d in zip(a, ab)]
        assert math.dist((p.x, p.y, p.z), closest) < 1e-3
    end = _flange(robot.history[-1])
    assert math.dist((end.x, end.y, end.z), b) < 1e-3


def test_pick_closes_with_the_body_center_at_the_grasp_offset_and_lifts():
    manipulator, robot, gripper = _manipulator()
    state = manipulator.pick("cubo", _cube())

    assert state.holding_object
    closing = [config for opening, config in gripper.commands if opening == 1.0]
    assert len(closing) == 1
    at_close = _flange(closing[0])
    assert (at_close.x, at_close.y, at_close.z) == pytest.approx((-0.571, -0.141, 0.025 + 0.14), abs=1e-3)
    end = _flange(robot.configuration)
    assert end.z == pytest.approx(0.025 + 0.14 + _SETTINGS.lift_distance, abs=1e-3)


def test_pick_opens_before_approaching():
    manipulator, _, gripper = _manipulator()
    manipulator.pick("cubo", _cube())
    assert gripper.commands[0][0] == 0.0


def test_a_failed_grasp_reopens_retreats_and_raises():
    manipulator, robot, gripper = _manipulator(grabs=False)
    with pytest.raises(GraspFailedError):
        manipulator.pick("cubo", _cube())
    assert gripper.opening == 0.0
    end = _flange(robot.configuration)
    assert end.z == pytest.approx(0.025 + 0.14 + _SETTINGS.approach_distance, abs=1e-3)


def test_saying_no_stops_before_going_down():
    manipulator, robot, gripper = _manipulator(confirm=lambda message: False)
    with pytest.raises(OperationCancelledError):
        manipulator.pick("cubo", _cube())
    lowest = min(_flange(c).z for c in robot.history)
    assert lowest > 0.025 + 0.14 + _SETTINGS.approach_distance - 1e-3
    assert all(opening == 0.0 for opening, _ in gripper.commands)


def test_a_fixed_body_is_rejected_without_moving():
    manipulator, robot, gripper = _manipulator()
    with pytest.raises(ValueError):
        manipulator.pick("mesa", _cube(graspable=False))
    assert robot.history == [robot.configuration] and gripper.commands == []


def test_place_opens_with_the_center_at_the_target_and_retreats():
    manipulator, robot, gripper = _manipulator()
    manipulator.pick("cubo", _cube())
    manipulator.place(Point(-0.52, 0.10, 0.025))

    opening = gripper.commands[-1]
    assert opening[0] == 0.0
    at_release = _flange(opening[1])
    assert (at_release.x, at_release.y, at_release.z) == pytest.approx((-0.52, 0.10, 0.165), abs=1e-3)
    end = _flange(robot.configuration)
    assert end.z == pytest.approx(0.165 + _SETTINGS.approach_distance, abs=1e-3)



# --- Orientación hacia abajo y semillas de IK (01/10) -----------------------


def test_a_tool_already_looking_down_keeps_its_orientation():
    manipulator, _, _ = _manipulator()
    down = manipulator.flange_pose()
    assert top_down_quaternion(down) == (down.qx, down.qy, down.qz, down.qw)


def test_from_home_the_tool_is_turned_to_look_down():
    """El fallo del 01/10: en home la herramienta mira en horizontal y
    "encima" salía de lado. Ahora la pose de agarre mira hacia abajo."""
    home = _flange(_configuration([0] * 6))
    assert tool_axis(home)[2] == pytest.approx(0, abs=1e-6)  # horizontal
    qx, qy, qz, qw = top_down_quaternion(home)
    assert tool_axis(Pose(0, 0, 0, qx, qy, qz, qw)) == pytest.approx((0, 0, -1), abs=1e-9)


def test_grasp_pose_from_home_is_above_the_point_looking_down():
    robot = FakeRobot(_configuration([0] * 6))
    manipulator = Manipulator(robot, FakeGripper(robot), _KINEMATICS, _SETTINGS, log=lambda m: None)
    grasp = manipulator.grasp_pose_for(Point(-0.5, 0.2, 0.025))
    assert (grasp.x, grasp.y, grasp.z) == pytest.approx((-0.5, 0.2, 0.025 + 0.14))
    assert tool_axis(grasp) == pytest.approx((0, 0, -1), abs=1e-9)


class _OnlyFromSeed:
    """Cinemática que solo converge si parte de `seed` (como PoE desde el
    borde del alcance)."""

    def __init__(self, seed):
        self.seed = seed

    def forward_kinematics(self, configuration):
        return _KINEMATICS.forward_kinematics(configuration)

    def compute_trajectory(self, goal, current):
        if current != self.seed:
            raise RuntimeError("no convergió")
        return _KINEMATICS.compute_trajectory(goal, current)


def test_move_to_pose_retries_from_known_postures_when_ik_fails_from_here():
    seed = _configuration(_WORK_DEGREES)
    robot = FakeRobot(_configuration([0] * 6))
    manipulator = Manipulator(robot, None, _OnlyFromSeed(seed), _SETTINGS, log=lambda m: None, ik_seeds=[seed])
    goal = _flange(_configuration([0.0, 25.0, 100.0, -35.0, -90.0, 0.0]))
    manipulator.move_to_pose(goal)
    end = _flange(robot.configuration)
    assert (end.x, end.y, end.z) == pytest.approx((goal.x, goal.y, goal.z), abs=1e-3)


def test_without_any_solution_the_error_says_it_is_probably_out_of_reach():
    seed = _configuration(_WORK_DEGREES)
    robot = FakeRobot(_configuration([0] * 6))
    manipulator = Manipulator(robot, None, _OnlyFromSeed("ninguna"), _SETTINGS, log=lambda m: None, ik_seeds=[seed])
    with pytest.raises(RuntimeError, match="fuera de alcance"):
        manipulator.move_to_pose(Pose(3.0, 0, 0))
