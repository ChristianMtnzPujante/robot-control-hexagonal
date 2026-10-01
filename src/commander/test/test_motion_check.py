"""Tests del ensayo en seco, la revisión de movimientos, la elección de la
solución de IK más cercana y el ciclo de `real_cell_check` (con la
cinemática PoE real y un robot de mentira)."""

import math
from types import SimpleNamespace

import pytest

from commander.manipulation import Manipulator
from commander.motion_check import MotionLimits, analyze, compare
from commander.real_cell_check import Abort, Check
from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory

from test_manipulation import _KINEMATICS, _SETTINGS, _WORK_DEGREES, FakeGripper, FakeRobot, _configuration, _flange

_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _manipulator(degrees=_WORK_DEGREES, kinematics=_KINEMATICS, seeds=()):
    robot = FakeRobot(_configuration(degrees))
    manipulator = Manipulator(robot, FakeGripper(robot), kinematics, _SETTINGS, log=lambda m: None, ik_seeds=seeds)
    return manipulator, robot


def test_a_dry_run_records_exactly_what_the_real_one_sends_and_moves_nothing():
    manipulator, robot = _manipulator()
    start = robot.configuration
    goal = _flange(_configuration([10, 25, 100, -35, -90, 10]))
    twin, recorder = manipulator.dry_run()
    twin.move_to_pose(goal)
    assert robot.configuration == start and robot.history == [start]
    manipulator.move_to_pose(goal)
    assert robot.history[1:] == recorder.waypoints


def test_analyze_reports_steps_travel_and_lowest_tool_point():
    manipulator, _ = _manipulator()
    start = manipulator.current_configuration()
    twin, recorder = manipulator.dry_run()
    twin.move_joints(_configuration([0, 0, 90, 0, -90, 0]))
    report = analyze(start, recorder.waypoints, _KINEMATICS, 0.14, MotionLimits(max_step_deg=5, min_tool_z=0.05))
    assert report.ok and report.waypoints == 51  # straight_line incluye el de partida
    assert report.travel_deg["joint2"] == pytest.approx(-20) and report.max_step_deg < 1
    assert report.min_tool_z == pytest.approx(0.265 - 0.14, abs=2e-3)  # el punto de agarre al empezar


def test_analyze_flags_big_jumps_and_going_too_low():
    start = _configuration(_WORK_DEGREES)
    jump = _configuration([20] + _WORK_DEGREES[1:])
    report = analyze(start, [jump], _KINEMATICS, 0.14, MotionLimits(max_step_deg=5, min_tool_z=0.5))
    assert not report.ok
    assert any("salto de 20.0° en joint1" in p for p in report.problems)
    assert any("baja a z" in p for p in report.problems)


def test_compare_is_zero_for_the_same_configuration_and_grows_with_error():
    a = _configuration(_WORK_DEGREES)
    assert compare(a, a, _KINEMATICS) == (0.0, 0.0)
    joint_error, position_error = compare(a, _configuration([1] + _WORK_DEGREES[1:]), _KINEMATICS)
    assert joint_error == pytest.approx(1) and position_error > 5


class _FarFromHere:
    """Desde la configuración actual converge a una solución con vueltas de
    más y en otra rama (como PoE desde la home del CR5); desde la semilla,
    a la buena."""

    def __init__(self, far, near, seed):
        self.far, self.near, self.seed = far, near, seed

    def forward_kinematics(self, configuration):
        return _KINEMATICS.forward_kinematics(configuration)

    def compute_trajectory(self, goal, current):
        return Trajectory.straight_line(current, self.near if current == self.seed else self.far, 5)


def test_move_to_pose_picks_the_solution_that_moves_least_and_unwraps_turns():
    seed = _configuration([0, 20, 100, -30, -90, 0])
    far = _configuration([207.7, 309.4, -39.1, -180.3, -90.0, 117.7])  # el caso real del 01/10
    near = _configuration([0, 25, 100, -35, -90, 0])
    manipulator, robot = _manipulator(degrees=[0] * 6, kinematics=_FarFromHere(far, near, seed), seeds=[seed])
    manipulator.move_to_pose(Pose(0, 0, 0))
    end = robot.configuration
    assert [round(math.degrees(end.angle_of(j)), 6) for j in _JOINTS] == [0, 25, 100, -35, -90, 0]
    steps = [abs(math.degrees(b.angle_of(j) - a.angle_of(j)))
             for a, b in zip(robot.history, robot.history[1:]) for j in _JOINTS]
    assert max(steps) <= 2.0 + 1e-9


def test_a_solution_one_turn_away_comes_back_to_the_near_turn():
    near_turn = _configuration([370, 20, 100, -30, -90, 0])  # 370° = 10°
    manipulator, robot = _manipulator(degrees=[0, 20, 100, -30, -90, 0],
                                      kinematics=_FarFromHere(near_turn, near_turn, None))
    manipulator.move_to_pose(Pose(0, 0, 0))
    assert math.degrees(robot.configuration.angle_of("joint1")) == pytest.approx(10)


# --- El ciclo de real_cell_check ------------------------------------------------


def _check(confirm=lambda m: True, min_tool_z=None):
    manipulator, robot = _manipulator()
    handle = SimpleNamespace(manipulator=manipulator)
    args = SimpleNamespace(max_step=5.0, clearance=0.10)
    return Check(handle, args, confirm, table_z=0.0), robot


def test_a_plan_that_breaks_the_limits_stops_before_moving():
    check, robot = _check()
    start = robot.configuration
    with pytest.raises(Abort, match="no cumple"):
        check.move("bajar demasiado", lambda m: m.move_joints(_configuration([0, 60, 110, -80, -90, 0])), 0.25)
    assert robot.configuration == start
    assert any(line.startswith("PROBLEMA") for line in check.report[-1]["planned"])


def test_saying_no_stops_before_moving():
    check, robot = _check(confirm=lambda m: False)
    start = robot.configuration
    with pytest.raises(Abort, match="cancelado"):
        check.move("postura", lambda m: m.move_joints(_configuration([0, 0, 90, 0, -90, 0])), 0.1)
    assert robot.configuration == start and check.report[-1]["executed"] is False


def test_an_accepted_move_is_executed_and_measured():
    check, robot = _check()
    check.move("postura", lambda m: m.move_joints(_configuration([0, 0, 90, 0, -90, 0])), 0.1)
    entry = check.report[-1]
    assert entry["executed"] and entry["joint_error_deg"] == 0 and entry["position_error_mm"] == 0
    assert math.degrees(robot.configuration.angle_of("joint3")) == pytest.approx(90)
