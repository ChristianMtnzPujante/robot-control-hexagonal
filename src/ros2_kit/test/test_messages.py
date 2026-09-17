from shared_kernel import Plane, Point, Scene, SphereObstacle

from ros2_kit import (
    from_obstacle_report_msg,
    from_object_report_msg,
    from_scene_msg,
    to_obstacle_report_msg,
    to_object_report_msg,
    to_scene_msg,
)


def test_empty_scene_round_trips():
    scene = Scene.empty()

    assert from_scene_msg(to_scene_msg(scene)) == scene


def test_full_scene_round_trips():
    scene = (
        Scene.empty()
        .with_plane("mesa", Plane(point=Point(0, 0, 0), normal=Point(0, 0, 1)))
        .with_obstacle("caja", SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05))
        .with_object("objetivo", Point(0.25, 0.25, 0.50))
    )

    assert from_scene_msg(to_scene_msg(scene)) == scene


def test_to_scene_msg_is_plain_json_in_a_string():
    import json

    scene = Scene.empty().with_object("objetivo", Point(1.0, 2.0, 3.0))

    msg = to_scene_msg(scene)

    assert json.loads(msg.data)["objects"]["objetivo"] == [1.0, 2.0, 3.0]


def test_obstacle_report_round_trips_with_its_name():
    obstacle = SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05)

    name, round_tripped = from_obstacle_report_msg(to_obstacle_report_msg("caja", obstacle))

    assert name == "caja"
    assert round_tripped == obstacle


def test_object_report_round_trips_with_its_name():
    position = Point(0.4, 0.1, 0.5)

    name, round_tripped = from_object_report_msg(to_object_report_msg("objetivo", position))

    assert name == "objetivo"
    assert round_tripped == position
