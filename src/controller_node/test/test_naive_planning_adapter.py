from shared_kernel import JointConfiguration, JointPosition, Pose, Scene, Trajectory

from controller_node.adapters.naive_planning_adapter import NaivePlanningAdapter

_CONFIGURATION = JointConfiguration.create(
    [JointPosition("joint1", 0.0)]
).value


class _StubKinematicsPort:
    def __init__(self) -> None:
        self.received_goal = None
        self.received_configuration = None

    def compute_trajectory(self, goal: Pose, current_configuration: JointConfiguration) -> Trajectory:
        self.received_goal = goal
        self.received_configuration = current_configuration
        return Trajectory.straight_line(current_configuration, current_configuration, steps=1)


def test_delegates_to_the_wrapped_kinematics_port_ignoring_the_scene():
    kinematics = _StubKinematicsPort()
    adapter = NaivePlanningAdapter(kinematics)
    goal = Pose(x=0.1, y=0.2, z=0.3)

    trajectory = adapter.compute_trajectory(goal, _CONFIGURATION, Scene.empty())

    assert kinematics.received_goal is goal
    assert kinematics.received_configuration is _CONFIGURATION
    assert trajectory.waypoints
