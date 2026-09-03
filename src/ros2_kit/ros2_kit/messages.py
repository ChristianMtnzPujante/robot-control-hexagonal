"""Traducción entre mensajes estándar de ROS2 y el vocabulario de
shared_kernel. Es infraestructura, no dominio: por eso vive aquí y no en
shared_kernel, que debe seguir sin saber que ROS2 existe.

Es el único sitio donde robot_node/controller_node/perception_node
deberían tocar sensor_msgs/geometry_msgs/std_msgs directamente para estas
conversiones -- antes estaba duplicado en los node.py.

`to_scene_msg`/`from_scene_msg` serializan `Scene` como JSON dentro de
`std_msgs/String` (no un `.msg` propio) -- mismo patrón que `<ns>/feedback`
en `controller_node`, decisión tomada al cablear `perception_node` (ver
ROADMAP.md Bloque 3 y docs/nodos_ros2.md §4).
"""

from __future__ import annotations

import json

from geometry_msgs.msg import Pose as PoseMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from shared_kernel import JointConfiguration, JointPosition, Plane, Point, Pose, Scene, SphereObstacle


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


def _point_to_list(point: Point) -> list:
    return [point.x, point.y, point.z]


def _point_from_list(values: list) -> Point:
    x, y, z = values
    return Point(x, y, z)


def to_scene_msg(scene: Scene) -> String:
    """JSON en `std_msgs/String`, mismo patrón que ya usa `<ns>/feedback`
    -- sin paquete de interfaces `.msg` propio (decisión tomada al cablear
    `perception_node`, ver ROADMAP.md Bloque 3 y docs/nodos_ros2.md §4)."""
    payload = {
        "planes": {
            name: {
                "point": _point_to_list(plane.point),
                "normal": _point_to_list(plane.normal),
            }
            for name, plane in scene.planes.items()
        },
        "obstacles": {
            name: {
                "center": _point_to_list(obstacle.center),
                "radius": obstacle.radius,
            }
            for name, obstacle in scene.obstacles.items()
        },
        "objects": {
            name: _point_to_list(point) for name, point in scene.objects.items()
        },
    }
    return String(data=json.dumps(payload))


def from_scene_msg(msg: String) -> Scene:
    payload = json.loads(msg.data)
    scene = Scene.empty()
    for name, plane in payload.get("planes", {}).items():
        scene = scene.with_plane(
            name,
            Plane(
                point=_point_from_list(plane["point"]),
                normal=_point_from_list(plane["normal"]),
            ),
        )
    for name, obstacle in payload.get("obstacles", {}).items():
        scene = scene.with_obstacle(
            name,
            SphereObstacle(
                center=_point_from_list(obstacle["center"]), radius=obstacle["radius"]
            ),
        )
    for name, point in payload.get("objects", {}).items():
        scene = scene.with_object(name, _point_from_list(point))
    return scene
