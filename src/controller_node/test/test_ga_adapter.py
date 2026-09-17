"""Tests de GaKinematicsAdapter (gafro/pygafro). Requieren `pygafro`
instalado (rueda de PyPI); si no está, se saltan todos con el motivo."""

import math

import numpy as np
import pytest

from shared_kernel import (
    JointConfiguration,
    JointDescription,
    JointPosition,
    Pose,
    RobotDescription,
)

pytest.importorskip("pygafro", reason="pygafro no instalado: pip install pygafro")

from controller_node.adapters.ga_adapter import GaKinematicsAdapter  # noqa: E402
from controller_node.adapters.poe_adapter import (  # noqa: E402
    PoeKinematicsAdapter,
    _DEFAULT_CR5_DESCRIPTION,
)

_JOINT_NAMES = _DEFAULT_CR5_DESCRIPTION.joint_names


def _configuration(names, values) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, float(value)) for name, value in zip(names, values)]
    ).value


def _pose_vector(pose: Pose) -> np.ndarray:
    return np.array([pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw])


def _same_pose(a: Pose, b: Pose, atol: float = 1e-9) -> bool:
    va, vb = _pose_vector(a), _pose_vector(b)
    # q y -q son la misma rotación.
    return np.allclose(va[:3], vb[:3], atol=atol) and (
        np.allclose(va[3:], vb[3:], atol=atol) or np.allclose(va[3:], -vb[3:], atol=atol)
    )


def test_ga_forward_kinematics_matches_poe_on_random_configurations():
    """El núcleo de F1.1: mismos datos crudos (RobotDescription), dos
    álgebras, misma cinemática directa -- a precisión máquina."""
    ga, poe = GaKinematicsAdapter(), PoeKinematicsAdapter()
    rng = np.random.default_rng(0)
    for _ in range(100):
        configuration = _configuration(_JOINT_NAMES, rng.uniform(-math.pi, math.pi, 6))
        assert _same_pose(ga.forward_kinematics(configuration), poe.forward_kinematics(configuration))


def test_ga_link_poses_match_poe_for_every_joint():
    ga, poe = GaKinematicsAdapter(), PoeKinematicsAdapter()
    rng = np.random.default_rng(1)
    for _ in range(20):
        configuration = _configuration(_JOINT_NAMES, rng.uniform(-math.pi, math.pi, 6))
        ga_poses, poe_poses = ga.link_poses(configuration), poe.link_poses(configuration)
        assert len(ga_poses) == len(poe_poses) == 6
        for ga_pose, poe_pose in zip(ga_poses, poe_poses):
            assert _same_pose(ga_pose, poe_pose)
        assert ga_poses[-1] == ga.forward_kinematics(configuration)


def test_ga_adapter_without_args_converges_for_cr5_default():
    """Espejo de test_poe_adapter_without_args_still_converges_for_cr5_default:
    IK real hacia una pose generada por la propia FK, desde la home."""
    adapter = GaKinematicsAdapter()
    home = _configuration(_JOINT_NAMES, np.zeros(6))
    target = np.array([0.1, -0.1, 0.15, 0.05, -0.2, 0.1])
    goal = adapter.forward_kinematics(_configuration(_JOINT_NAMES, target))

    trajectory = adapter.compute_trajectory(goal, home)
    final = trajectory.waypoints[-1]

    assert trajectory.waypoints[0] == home
    assert len(trajectory.waypoints) == 21  # steps=20 tramos -> 21 puntos
    for name, expected in zip(_JOINT_NAMES, target):
        assert final.angle_of(name) == pytest.approx(expected, abs=1e-2)
    assert adapter.last_iteration_count is not None and adapter.last_iteration_count < 50
    # La pose alcanzada cumple las tolerancias del propio adaptador.
    reached = adapter.forward_kinematics(final)
    assert np.linalg.norm(_pose_vector(reached)[:3] - _pose_vector(goal)[:3]) < 1e-4


def test_ga_and_poe_reach_the_same_pose_from_the_same_start():
    """Ambas IK parten de la misma configuración y convergen a la misma
    rama (mismo esquema Newton-Raphson amortiguado): las soluciones
    articulares coinciden dentro de la tolerancia de pose."""
    ga, poe = GaKinematicsAdapter(steps=1), PoeKinematicsAdapter(steps=1)
    start = _configuration(_JOINT_NAMES, [0.2, -0.3, 0.4, 0.1, -0.2, 0.3])
    goal = poe.forward_kinematics(_configuration(_JOINT_NAMES, [0.3, -0.4, 0.55, 0.2, -0.1, 0.25]))
    ga_final = ga.compute_trajectory(goal, start).waypoints[-1]
    poe_final = poe.compute_trajectory(goal, start).waypoints[-1]
    for name in _JOINT_NAMES:
        assert ga_final.angle_of(name) == pytest.approx(poe_final.angle_of(name), abs=5e-3)


def _two_dof_planar_description() -> RobotDescription:
    joints = [
        JointDescription(
            name="j1", joint_type="revolute",
            origin_xyz=(0.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0),
        ),
        JointDescription(
            name="j2", joint_type="revolute",
            origin_xyz=(1.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0),
        ),
    ]
    return RobotDescription.create(joints, base_link="base", tip_link="j2").value


def test_ga_forward_kinematics_matches_the_closed_form_for_two_dof_robot():
    adapter = GaKinematicsAdapter(robot_description=_two_dof_planar_description())
    theta1, theta2 = 0.4, -0.7
    pose = adapter.forward_kinematics(_configuration(("j1", "j2"), (theta1, theta2)))
    phi = theta1 + theta2
    assert pose.x == pytest.approx(math.cos(theta1), abs=1e-9)
    assert pose.y == pytest.approx(math.sin(theta1), abs=1e-9)
    assert pose.z == pytest.approx(0.0, abs=1e-9)
    assert abs(pose.qz) == pytest.approx(abs(math.sin(phi / 2)), abs=1e-9)
    assert abs(pose.qw) == pytest.approx(abs(math.cos(phi / 2)), abs=1e-9)


def test_ga_adapter_converges_for_two_dof_synthetic_robot():
    adapter = GaKinematicsAdapter(robot_description=_two_dof_planar_description(), steps=1)
    target_theta1, target_theta2 = 0.6, -0.9
    goal = adapter.forward_kinematics(_configuration(("j1", "j2"), (target_theta1, target_theta2)))
    final = adapter.compute_trajectory(goal, _configuration(("j1", "j2"), (0.0, 0.0))).waypoints[-1]
    assert final.angle_of("j1") == pytest.approx(target_theta1, abs=1e-3)
    assert final.angle_of("j2") == pytest.approx(target_theta2, abs=1e-3)


def test_ga_adapter_supports_prismatic_joints_like_poe():
    """Misma descripción sintética con una prismática: GA y PoE deben dar
    la misma FK (verifica la convención TranslatorGenerator = eje directo)."""
    joints = [
        JointDescription(
            name="spin", joint_type="revolute",
            origin_xyz=(0.0, 0.0, 0.2), origin_rpy=(0.3, 0.0, 0.0), axis=(0.0, 1.0, 0.0),
        ),
        JointDescription(
            name="slide", joint_type="prismatic",
            origin_xyz=(0.5, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0),
        ),
    ]
    description = RobotDescription.create(joints, base_link="base", tip_link="tip").value
    ga = GaKinematicsAdapter(robot_description=description)
    poe = PoeKinematicsAdapter(robot_description=description)
    for values in ((0.0, 0.0), (0.7, 0.25), (-1.1, -0.4)):
        configuration = _configuration(("spin", "slide"), values)
        assert _same_pose(ga.forward_kinematics(configuration), poe.forward_kinematics(configuration))


def test_ga_adapter_raises_when_goal_is_unreachable():
    adapter = GaKinematicsAdapter(max_iterations=30)
    home = _configuration(_JOINT_NAMES, np.zeros(6))
    far_away = Pose(x=5.0, y=5.0, z=5.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    with pytest.raises(RuntimeError, match="no convergió"):
        adapter.compute_trajectory(far_away, home)
