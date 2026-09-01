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

from controller_node.adapters.poe_adapter import (
    PoeKinematicsAdapter,
    _DEFAULT_CR5_DESCRIPTION,
    _forward_kinematics,
    _screw_axes_from_description,
)


def _matrix_to_quaternion(rotation: np.ndarray):
    """Shepperd's method -- solo para construir goals de test a partir de
    una matriz de rotación conocida, no se usa en producción."""
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def test_default_cr5_description_matches_previous_hardcoded_values():
    assert _DEFAULT_CR5_DESCRIPTION.joint_names == (
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    )
    assert _DEFAULT_CR5_DESCRIPTION.degrees_of_freedom == 6
    assert all(joint.joint_type == "revolute" for joint in _DEFAULT_CR5_DESCRIPTION.joints)


def test_poe_adapter_without_args_still_converges_for_cr5_default():
    """Regresión: PoeKinematicsAdapter() (como la construye
    controller_node/_build_adapter, sin argumentos) debe seguir funcionando
    igual que antes del refactor a RobotDescription."""
    adapter = PoeKinematicsAdapter()
    home = JointConfiguration.create(
        [JointPosition(name, 0.0) for name in _DEFAULT_CR5_DESCRIPTION.joint_names]
    ).value

    target = np.array([0.1, -0.1, 0.15, 0.05, -0.2, 0.1])
    screw_axes, home_pose = _screw_axes_from_description(_DEFAULT_CR5_DESCRIPTION)
    goal_transform = _forward_kinematics(screw_axes, home_pose, target)
    qx, qy, qz, qw = _matrix_to_quaternion(goal_transform[:3, :3])
    goal = Pose(
        x=goal_transform[0, 3], y=goal_transform[1, 3], z=goal_transform[2, 3],
        qx=qx, qy=qy, qz=qz, qw=qw,
    )

    trajectory = adapter.compute_trajectory(goal, home)
    final = trajectory.waypoints[-1]

    # Tolerancia más laxa que en los sintéticos: el criterio de convergencia
    # de compute_trajectory es sobre el error de pose (posición/orientación),
    # no sobre distancia en espacio de articulaciones -- con 6 DOF reales
    # (vs. la solución única de los sintéticos de 1-2 DOF) un residuo de
    # pose dentro de tolerancia puede corresponder a unos pocos mrad de
    # diferencia articular.
    for name, expected in zip(_DEFAULT_CR5_DESCRIPTION.joint_names, target):
        assert final.angle_of(name) == pytest.approx(expected, abs=1e-2)


def _two_dof_planar_description() -> RobotDescription:
    """Brazo planar de 2 revolutas, longitud de eslabón 1: joint1 en el
    origen, joint2 desplazado (1,0,0) respecto de joint1. El efector es la
    propia joint2 (sin eslabón adicional), así que la posición alcanzable
    depende solo de theta1 (círculo de radio 1) y la orientación de
    theta1+theta2 -- suficiente para validar la generalización N=2 del
    Jacobiano (antes fijo a 6x6) con una solución única y verificable a
    mano."""
    joints = [
        JointDescription(
            name="j1", joint_type="revolute",
            origin_xyz=(0.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0),
            axis=(0.0, 0.0, 1.0),
        ),
        JointDescription(
            name="j2", joint_type="revolute",
            origin_xyz=(1.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0),
            axis=(0.0, 0.0, 1.0),
        ),
    ]
    return RobotDescription.create(joints, base_link="base", tip_link="j2").value


def test_poe_adapter_converges_for_two_dof_synthetic_robot():
    """FK cerrada a mano para este robot (ver docstring de
    _two_dof_planar_description): posición=(cos theta1, sin theta1, 0),
    orientación=Rz(theta1+theta2). Ejercita jacobian=(6,2), no (6,6)."""
    description = _two_dof_planar_description()
    adapter = PoeKinematicsAdapter(robot_description=description)

    target_theta1, target_theta2 = 0.4, 0.3
    phi = target_theta1 + target_theta2
    goal = Pose(
        x=math.cos(target_theta1), y=math.sin(target_theta1), z=0.0,
        qx=0.0, qy=0.0, qz=math.sin(phi / 2), qw=math.cos(phi / 2),
    )
    start = JointConfiguration.create(
        [JointPosition("j1", 0.35), JointPosition("j2", 0.25)]
    ).value

    trajectory = adapter.compute_trajectory(goal, start)
    final = trajectory.waypoints[-1]

    assert final.angle_of("j1") == pytest.approx(target_theta1, abs=1e-3)
    assert final.angle_of("j2") == pytest.approx(target_theta2, abs=1e-3)


def test_link_poses_matches_closed_form_for_two_dof_robot():
    """FK cerrada a mano por articulación (mismo robot que el test de
    arriba): joint1 no se mueve por su propia rotación (sigue en el
    origen, solo cambia su orientación a Rz(theta1)); joint2 arranca en
    (1,0,0) local y queda en Rz(theta1)·(1,0,0) tras la rotación de
    joint1, con orientación Rz(theta1+theta2). La última pose debe
    coincidir exactamente con forward_kinematics (mismo tip, sin eslabón
    extra tras joint2)."""
    description = _two_dof_planar_description()
    adapter = PoeKinematicsAdapter(robot_description=description)

    theta1, theta2 = 0.4, 0.3
    configuration = JointConfiguration.create(
        [JointPosition("j1", theta1), JointPosition("j2", theta2)]
    ).value

    poses = adapter.link_poses(configuration)
    assert len(poses) == 2

    joint1_pose, joint2_pose = poses
    assert joint1_pose.x == pytest.approx(0.0, abs=1e-9)
    assert joint1_pose.y == pytest.approx(0.0, abs=1e-9)
    assert joint1_pose.qz == pytest.approx(math.sin(theta1 / 2), abs=1e-9)
    assert joint1_pose.qw == pytest.approx(math.cos(theta1 / 2), abs=1e-9)

    assert joint2_pose.x == pytest.approx(math.cos(theta1), abs=1e-9)
    assert joint2_pose.y == pytest.approx(math.sin(theta1), abs=1e-9)
    phi = theta1 + theta2
    assert joint2_pose.qz == pytest.approx(math.sin(phi / 2), abs=1e-9)
    assert joint2_pose.qw == pytest.approx(math.cos(phi / 2), abs=1e-9)

    tip = adapter.forward_kinematics(configuration)
    assert poses[-1] == tip


def _single_prismatic_description() -> RobotDescription:
    joints = [
        JointDescription(
            name="slide", joint_type="prismatic",
            origin_xyz=(0.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0),
            axis=(0.0, 0.0, 1.0),
        ),
    ]
    return RobotDescription.create(joints, base_link="base", tip_link="slide").value


def test_forward_kinematics_matches_the_closed_form_pose_for_two_dof_robot():
    """Redondo con la FK cerrada a mano de _two_dof_planar_description:
    forward_kinematics(thetas) debe reproducir exactamente el mismo Pose
    que se usó como goal en test_poe_adapter_converges_for_two_dof_synthetic_robot."""
    description = _two_dof_planar_description()
    adapter = PoeKinematicsAdapter(robot_description=description)

    theta1, theta2 = 0.4, 0.3
    phi = theta1 + theta2
    configuration = JointConfiguration.create(
        [JointPosition("j1", theta1), JointPosition("j2", theta2)]
    ).value

    pose = adapter.forward_kinematics(configuration)

    assert pose.x == pytest.approx(math.cos(theta1), abs=1e-9)
    assert pose.y == pytest.approx(math.sin(theta1), abs=1e-9)
    assert pose.z == pytest.approx(0.0, abs=1e-9)
    assert pose.qz == pytest.approx(math.sin(phi / 2), abs=1e-9)
    assert pose.qw == pytest.approx(math.cos(phi / 2), abs=1e-9)


def test_poe_adapter_converges_for_prismatic_joint():
    """FK cerrada a mano: posición=(0,0,theta), orientación=identidad."""
    description = _single_prismatic_description()
    adapter = PoeKinematicsAdapter(robot_description=description)

    target = 0.5
    goal = Pose(x=0.0, y=0.0, z=target, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    start = JointConfiguration.create([JointPosition("slide", 0.1)]).value

    trajectory = adapter.compute_trajectory(goal, start)
    final = trajectory.waypoints[-1]

    assert final.angle_of("slide") == pytest.approx(target, abs=1e-3)
