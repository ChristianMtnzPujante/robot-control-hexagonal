import numpy as np

from shared_kernel import (
    JointConfiguration,
    JointPosition,
    Point,
    Pose,
    Scene,
    SphereObstacle,
    Trajectory,
)

from controller_node.adapters.obstacle_avoiding_planning_adapter import (
    ObstacleAvoidingPlanningAdapter,
)

_CONFIGURATION = JointConfiguration.create([JointPosition("joint1", 0.0)]).value


class _StubKinematicsPort:
    """Cinemática de prueba: `forward_kinematics` es una función lineal fija
    conocida (no depende de PoE de verdad) y `compute_trajectory` devuelve,
    igual que `Trajectory.straight_line` de verdad, un waypoint de arranque
    (la `current_configuration` recibida) más uno de llegada que "recuerda"
    el goal pedido -- necesario para poder comprobar tanto qué Pose le pidió
    el planificador en cada tramo como que la concatenación de tramos no
    duplica el waypoint de unión (ver `compute_trajectory` del adaptador).
    """

    def __init__(self, start: Pose) -> None:
        self._start = start
        self.received_goals: list = []

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        return self._start

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        self.received_goals.append(goal)
        target = JointConfiguration.create([JointPosition("joint1", goal.x)]).value
        return Trajectory.create([current_configuration, target]).value


def test_no_obstacle_delegates_directly_with_a_single_leg():
    kinematics = _StubKinematicsPort(start=Pose(x=0.0, y=0.0, z=0.0))
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=1.0, y=0.0, z=0.0)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, Scene.empty())

    assert kinematics.received_goals == [goal]
    assert len(trajectory.waypoints) == 2


def test_far_obstacle_does_not_trigger_a_detour():
    kinematics = _StubKinematicsPort(start=Pose(x=0.0, y=0.0, z=0.0))
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=1.0, y=0.0, z=0.0)
    scene = Scene.empty().with_obstacle(
        "lejano", SphereObstacle(center=Point(0.5, 5.0, 0.0), radius=0.1)
    )

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    assert kinematics.received_goals == [goal]
    assert len(trajectory.waypoints) == 2


def test_obstacle_on_the_path_inserts_a_via_point_clear_of_it():
    kinematics = _StubKinematicsPort(start=Pose(x=0.0, y=0.0, z=0.0))
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=1.0, y=0.0, z=0.0, qx=0.1, qy=0.2, qz=0.3, qw=0.9)
    obstacle = SphereObstacle(center=Point(0.5, 0.0, 0.0), radius=0.1)
    scene = Scene.empty().with_obstacle("obstaculo", obstacle)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    assert len(kinematics.received_goals) == 2
    via = kinematics.received_goals[0]
    assert kinematics.received_goals[1] is goal
    # El via-point queda al menos a radius+clearance del centro del
    # obstáculo.
    center = np.array([obstacle.center.x, obstacle.center.y, obstacle.center.z])
    via_xyz = np.array([via.x, via.y, via.z])
    assert np.linalg.norm(via_xyz - center) >= obstacle.radius + 0.05 - 1e-9
    # Conserva la orientación del goal (simplificación deliberada, ver
    # docstring del adaptador).
    assert (via.qx, via.qy, via.qz, via.qw) == (goal.qx, goal.qy, goal.qz, goal.qw)
    # Dos tramos de dos waypoints cada uno (start->via, via->goal),
    # concatenados sin duplicar el waypoint de unión: start, via, goal.
    assert len(trajectory.waypoints) == 3


def test_obstacle_centered_exactly_on_the_segment_still_produces_a_clear_via_point():
    # Caso degenerado: el centro del obstáculo cae justo sobre la recta, el
    # vector "lejos del centro" tiene norma ~0 -- comprueba el fallback
    # lateral en vez de dividir por cero.
    kinematics = _StubKinematicsPort(start=Pose(x=0.0, y=0.0, z=0.0))
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=1.0, y=0.0, z=0.0)
    obstacle = SphereObstacle(center=Point(0.5, 0.0, 0.0), radius=0.1)
    scene = Scene.empty().with_obstacle("obstaculo", obstacle)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    via = kinematics.received_goals[0]
    center = np.array([obstacle.center.x, obstacle.center.y, obstacle.center.z])
    via_xyz = np.array([via.x, via.y, via.z])
    assert np.linalg.norm(via_xyz - center) >= obstacle.radius + 0.05 - 1e-9
    assert len(trajectory.waypoints) == 3


def test_only_the_worst_intersecting_obstacle_is_avoided():
    kinematics = _StubKinematicsPort(start=Pose(x=0.0, y=0.0, z=0.0))
    planner = ObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=1.0, y=0.0, z=0.0)
    mild = SphereObstacle(center=Point(0.3, 0.0, 0.0), radius=0.05)
    severe = SphereObstacle(center=Point(0.6, 0.0, 0.0), radius=0.2)
    scene = Scene.empty().with_obstacle("mild", mild).with_obstacle("severe", severe)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    assert len(kinematics.received_goals) == 2
    via = kinematics.received_goals[0]
    via_xyz = np.array([via.x, via.y, via.z])
    severe_center = np.array([severe.center.x, severe.center.y, severe.center.z])
    assert np.linalg.norm(via_xyz - severe_center) >= severe.radius + 0.05 - 1e-9
    assert len(trajectory.waypoints) == 3
