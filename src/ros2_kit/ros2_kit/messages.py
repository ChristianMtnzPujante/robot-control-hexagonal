"""Traducción entre mensajes estándar de ROS2 y el vocabulario de
shared_kernel. Es infraestructura, no dominio: por eso vive aquí y no en
shared_kernel, que debe seguir sin saber que ROS2 existe.

Es el único sitio donde robot_node y controller_node deberían tocar
sensor_msgs/geometry_msgs directamente para estas conversiones -- antes
estaba duplicado en los dos node.py.
"""

from __future__ import annotations

from geometry_msgs.msg import Pose as PoseMsg
from sensor_msgs.msg import JointState

from shared_kernel import JointConfiguration, JointPosition, Pose


def to_joint_configuration(msg: JointState) -> JointConfiguration:
    positions = [
        JointPosition(name, angle) for name, angle in zip(msg.name, msg.position)
    ]
    result = JointConfiguration.create(positions)
    if result.is_left():
        raise ValueError(str(result.value))
    return result.value


def to_joint_state_msg(configuration: JointConfiguration) -> JointState:
    msg = JointState()
    msg.name = [p.joint_name for p in configuration.positions]
    msg.position = [p.angle_radians for p in configuration.positions]
    return msg


def to_pose(msg: PoseMsg) -> Pose:
    return Pose(
        x=msg.position.x,
        y=msg.position.y,
        z=msg.position.z,
        qx=msg.orientation.x,
        qy=msg.orientation.y,
        qz=msg.orientation.z,
        qw=msg.orientation.w,
    )


def to_pose_msg(pose: Pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x = pose.x
    msg.position.y = pose.y
    msg.position.z = pose.z
    msg.orientation.x = pose.qx
    msg.orientation.y = pose.qy
    msg.orientation.z = pose.qz
    msg.orientation.w = pose.qw
    return msg
