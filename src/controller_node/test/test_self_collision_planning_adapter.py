import math

import pytest

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene, Trajectory

from controller_node.adapters.self_collision_planning_adapter import (
    SelfCollisionAvoidingPlanningAdapter,
    SelfCollisionAwarePlanningAdapter,
    SelfCollisionError,
    find_self_collision,
    structural_adjacency_exclusions,
)

_JOINT_NAMES = ["p1_x", "p1_y", "p2_x", "p2_y", "p3_x", "p3_y"]


def _configuration(p1, p2, p3) -> JointConfiguration:
    values = {"p1_x": p1[0], "p1_y": p1[1], "p2_x": p2[0], "p2_y": p2[1], "p3_x": p3[0], "p3_y": p3[1]}
    return JointConfiguration.create(
        [JointPosition(name, values[name]) for name in _JOINT_NAMES]
    ).value


_HOME = _configuration((1.0, 0.0), (2.0, 0.0), (3.0, 0.0))


class _ThreeLinkStubKinematicsPort:
    """Brazo sintético de 3 eslabones (base->p1->p2->p3) en el plano XY --
    solo hace falta 3 para tener un par no consecutivo por índice que
    comprobar (segmento 0 = base->p1, segmento 2 = p2->p3)."""

    def __init__(self, target: JointConfiguration) -> None:
        self._target = target

    def compute_trajectory(self, goal: Pose, current_configuration: JointConfiguration) -> Trajectory:
        return Trajectory.create([current_configuration, self._target]).value

    def link_poses(self, configuration: JointConfiguration):
        return [
            Pose(x=configuration.angle_of("p1_x"), y=configuration.angle_of("p1_y"), z=0.0),
            Pose(x=configuration.angle_of("p2_x"), y=configuration.angle_of("p2_y"), z=0.0),
            Pose(x=configuration.angle_of("p3_x"), y=configuration.angle_of("p3_y"), z=0.0),
        ]


_HOME_EXCLUSIONS = structural_adjacency_exclusions(
    _ThreeLinkStubKinematicsPort(_HOME).link_poses(_HOME)
)


def test_structural_exclusions_cover_only_the_two_adjacent_pairs_in_a_normal_arm():
    # (0,1) comparten p1, (1,2) comparten p2 -- (0,2) NO comparte nada (p1 y
    # p2 están a distancia 1.0 en _HOME), así que debe seguir comprobándose.
    assert _HOME_EXCLUSIONS == frozenset({(0, 1), (1, 2)})


def test_structural_exclusions_also_catch_a_zero_length_intermediate_link():
    # Reproduce el hallazgo real del 08/09: el CR5 tiene joint1/joint2 en
    # el MISMO punto físico (offset cero entre ambos en el URDF) -- aquí,
    # p1==p2 (segmento 1 degenerado). El par (0,2), pese a no ser
    # consecutivo por índice, SÍ comparte extremo físico y debe excluirse
    # igual que un par adyacente normal.
    degenerate_reference = _configuration((1.0, 0.0), (1.0, 0.0), (3.0, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(degenerate_reference)

    exclusions = structural_adjacency_exclusions(kinematics.link_poses(degenerate_reference))

    assert (0, 2) in exclusions


def test_find_self_collision_returns_none_when_links_have_margin():
    kinematics = _ThreeLinkStubKinematicsPort(_HOME)
    link_poses = kinematics.link_poses(_HOME)

    assert find_self_collision(link_poses, link_radius_meters=0.08, excluded_pairs=_HOME_EXCLUSIONS) is None


def test_find_self_collision_detects_non_adjacent_overlap():
    # segmento 2 (p2->p3) se solapa por completo con segmento 0 (base->p1).
    target = _configuration((1.0, 0.0), (0.02, 0.0), (0.5, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(target)
    link_poses = kinematics.link_poses(target)

    collision = find_self_collision(link_poses, link_radius_meters=0.08, excluded_pairs=_HOME_EXCLUSIONS)

    assert collision is not None
    link_a, link_b, penetration = collision
    assert (link_a, link_b) == (0, 2)
    assert penetration == pytest.approx(0.16)  # 2*0.08 - distancia(0)


def test_adapter_returns_trajectory_unchanged_when_no_self_collision():
    target = _configuration((1.0, 0.0), (2.0, 1.0), (3.0, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(target)
    adapter = SelfCollisionAwarePlanningAdapter(kinematics, link_radius_meters=0.08)

    trajectory = adapter.compute_trajectory(Pose(x=0, y=0, z=0), _HOME, Scene.empty())

    assert trajectory.waypoints == [_HOME, target]


def test_adapter_rejects_a_trajectory_that_self_collides():
    target = _configuration((1.0, 0.0), (0.02, 0.0), (0.5, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(target)
    adapter = SelfCollisionAwarePlanningAdapter(kinematics, link_radius_meters=0.08)

    with pytest.raises(SelfCollisionError):
        adapter.compute_trajectory(Pose(x=0, y=0, z=0), _HOME, Scene.empty())


def test_avoiding_adapter_returns_trajectory_unchanged_when_no_self_collision():
    target = _configuration((1.0, 0.0), (2.0, 1.0), (3.0, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(target)
    adapter = SelfCollisionAvoidingPlanningAdapter(kinematics, link_radius_meters=0.08)

    trajectory = adapter.compute_trajectory(Pose(x=0, y=0, z=0), _HOME, Scene.empty())

    assert trajectory.waypoints == [_HOME, target]


def test_avoiding_adapter_still_rejects_when_no_nudge_can_help():
    # _ThreeLinkStubKinematicsPort.compute_trajectory ignora la semilla
    # (siempre devuelve el mismo target) -- ningún desplazamiento de
    # "muñeca" puede cambiar nada aquí, así que debe agotar la búsqueda y
    # rendirse igual que SelfCollisionAwarePlanningAdapter.
    target = _configuration((1.0, 0.0), (0.02, 0.0), (0.5, 0.0))
    kinematics = _ThreeLinkStubKinematicsPort(target)
    adapter = SelfCollisionAvoidingPlanningAdapter(
        kinematics, link_radius_meters=0.08, nudge_step_degrees=10.0, max_nudge_degrees=20.0
    )

    with pytest.raises(SelfCollisionError):
        adapter.compute_trajectory(Pose(x=0, y=0, z=0), _HOME, Scene.empty())


def test_wrist_joint_pair_is_none_with_fewer_than_three_joints():
    configuration = JointConfiguration.create(
        [JointPosition("a", 0.0), JointPosition("b", 0.0)]
    ).value

    assert SelfCollisionAvoidingPlanningAdapter._wrist_joint_pair(configuration) is None


def test_wrist_joint_pair_skips_the_middle_joint():
    configuration = JointConfiguration.create(
        [JointPosition(n, 0.0) for n in ["j1", "j2", "j3", "j4", "j5", "j6"]]
    ).value

    assert SelfCollisionAvoidingPlanningAdapter._wrist_joint_pair(configuration) == ("j4", "j6")


def test_avoiding_adapter_finds_a_self_collision_free_branch_for_the_real_incident():
    # Reproduce el incidente real del 08/09: cr5_semicircle_demo.py, radio
    # 0.15m, 10 puntos, disparó GetErrorID()=[76] (autocolisión real) en el
    # tramo final. SelfCollisionAwarePlanningAdapter (ver test de arriba,
    # implícito) lo habría rechazado; este adaptador debe en cambio
    # encontrar una rama distinta y completar el arco entero.
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    home = JointConfiguration.create([JointPosition(n, 0.0) for n in joint_names]).value
    kinematics = PoeKinematicsAdapter(steps=1)
    home_pose = kinematics.forward_kinematics(home)
    exclusions = structural_adjacency_exclusions(kinematics.link_poses(home))

    radius, num_points = 0.15, 10
    center_x, center_z = home_pose.x, home_pose.z - radius
    points = [
        Pose(
            x=center_x + radius * math.cos(math.pi / 2 + math.pi * i / (num_points - 1)),
            y=home_pose.y,
            z=center_z + radius * math.sin(math.pi / 2 + math.pi * i / (num_points - 1)),
            qx=home_pose.qx, qy=home_pose.qy, qz=home_pose.qz, qw=home_pose.qw,
        )
        for i in range(num_points)
    ]

    adapter = SelfCollisionAvoidingPlanningAdapter(kinematics, link_radius_meters=0.04)

    current = home
    reached = [current]
    for point in points[1:]:
        trajectory = adapter.compute_trajectory(point, current, Scene.empty())
        current = trajectory.waypoints[-1]
        reached.append(current)

    # Los 9 puntos se alcanzaron (sin SelfCollisionError) y el destino
    # cartesiano final coincide con el objetivo pedido.
    assert len(reached) == num_points
    final_pose = kinematics.forward_kinematics(reached[-1])
    assert final_pose.x == pytest.approx(points[-1].x, abs=1e-3)
    assert final_pose.z == pytest.approx(points[-1].z, abs=1e-3)

    # Y ninguna de las configuraciones alcanzadas colisiona de verdad.
    for configuration in reached:
        assert find_self_collision(kinematics.link_poses(configuration), 0.04, exclusions) is None
