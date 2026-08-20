import math

import pytest

from urdf_kit import UnsupportedUrdfChainError, parse_urdf_string

_SIMPLE_CHAIN = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_simple_all_revolute_chain():
    description = parse_urdf_string(_SIMPLE_CHAIN, base_link="base", tip_link="link2")

    assert description.joint_names == ("joint1", "joint2")
    assert description.base_link == "base"
    assert description.tip_link == "link2"
    assert description.joints[0].origin_xyz == (0.0, 0.0, 0.1)
    assert description.joints[0].axis == (0.0, 0.0, 1.0)


def test_omitted_axis_defaults_to_x_not_z():
    description = parse_urdf_string(_SIMPLE_CHAIN, base_link="base", tip_link="link2")

    assert description.joints[1].axis == (1.0, 0.0, 0.0)


_CHAIN_WITH_FIXED_JOINT = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <link name="mid"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="link1"/>
    <child link="mid"/>
    <origin xyz="0.2 0 0" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="mid"/>
    <child link="link2"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_fixed_joint_is_folded_into_next_movable_joint():
    description = parse_urdf_string(
        _CHAIN_WITH_FIXED_JOINT, base_link="base", tip_link="link2"
    )

    # Solo quedan las 2 articulaciones móviles: "mount" (fixed) desaparece,
    # su transform queda compuesto dentro de joint2.
    assert description.joint_names == ("joint1", "joint2")
    joint2 = description.joints[1]
    # T = Trans(0.2,0,0)*Rot(yaw=90°) seguido de Trans(0.3,0,0)*Rot(identidad):
    # traslación = (0.2,0,0) + Rz(90°)@(0.3,0,0) = (0.2, 0.3, 0); rotación = Rz(90°).
    assert joint2.origin_xyz == pytest.approx((0.2, 0.3, 0.0), abs=1e-9)
    assert joint2.origin_rpy == pytest.approx((0.0, 0.0, math.pi / 2), abs=1e-9)
    assert joint2.axis == pytest.approx((0.0, 1.0, 0.0))


_PRISMATIC_CHAIN = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <joint name="slider" type="prismatic">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_prismatic_joint_is_parsed():
    description = parse_urdf_string(_PRISMATIC_CHAIN, base_link="base", tip_link="link1")

    assert description.joints[0].joint_type == "prismatic"
    assert description.joints[0].axis == (0.0, 0.0, 1.0)


_DISCONNECTED_URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def test_reversed_direction_raises_unsupported_chain():
    with pytest.raises(UnsupportedUrdfChainError):
        parse_urdf_string(_DISCONNECTED_URDF, base_link="link1", tip_link="base")


_UNSUPPORTED_JOINT_TYPE_URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <joint name="floater" type="floating">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_unsupported_joint_type_raises():
    with pytest.raises(UnsupportedUrdfChainError):
        parse_urdf_string(_UNSUPPORTED_JOINT_TYPE_URDF, base_link="base", tip_link="link1")
