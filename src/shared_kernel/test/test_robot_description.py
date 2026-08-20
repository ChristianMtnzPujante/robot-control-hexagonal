import pytest

from shared_kernel import InvalidRobotDescriptionError, JointDescription, RobotDescription


def _two_joints():
    return [
        JointDescription(
            name="joint1",
            joint_type="revolute",
            origin_xyz=(0.0, 0.0, 0.1),
            origin_rpy=(0.0, 0.0, 0.0),
            axis=(0.0, 0.0, 1.0),
        ),
        JointDescription(
            name="joint2",
            joint_type="revolute",
            origin_xyz=(0.5, 0.0, 0.0),
            origin_rpy=(0.0, 0.0, 0.0),
            axis=(0.0, 1.0, 0.0),
        ),
    ]


def test_create_with_valid_joints_succeeds():
    result = RobotDescription.create(_two_joints(), base_link="base", tip_link="tip")

    assert result.is_right()
    description = result.value
    assert description.joint_names == ("joint1", "joint2")
    assert description.degrees_of_freedom == 2
    assert description.base_link == "base"
    assert description.tip_link == "tip"


def test_create_rejects_empty_joints():
    result = RobotDescription.create([], base_link="base", tip_link="tip")

    assert result.is_left()
    assert isinstance(result.value, InvalidRobotDescriptionError)


def test_create_rejects_duplicate_joint_names():
    joints = _two_joints()
    joints[1] = JointDescription(
        name="joint1",
        joint_type="revolute",
        origin_xyz=(0.5, 0.0, 0.0),
        origin_rpy=(0.0, 0.0, 0.0),
    )

    result = RobotDescription.create(joints, base_link="base", tip_link="tip")

    assert result.is_left()
    assert isinstance(result.value, InvalidRobotDescriptionError)


def test_create_rejects_null_axis():
    joints = _two_joints()
    joints[0] = JointDescription(
        name="joint1",
        joint_type="revolute",
        origin_xyz=(0.0, 0.0, 0.1),
        origin_rpy=(0.0, 0.0, 0.0),
        axis=(0.0, 0.0, 0.0),
    )

    result = RobotDescription.create(joints, base_link="base", tip_link="tip")

    assert result.is_left()
    assert isinstance(result.value, InvalidRobotDescriptionError)


@pytest.mark.parametrize("base_link,tip_link", [("", "tip"), ("base", ""), ("  ", "tip")])
def test_create_rejects_empty_link_names(base_link, tip_link):
    result = RobotDescription.create(_two_joints(), base_link=base_link, tip_link=tip_link)

    assert result.is_left()
    assert isinstance(result.value, InvalidRobotDescriptionError)
