from shared_kernel import Plane, Point, Scene, SphereObstacle

from perception_node.adapters.static_perception_adapter import StaticPerceptionAdapter


def test_returns_exactly_the_scene_given_at_construction():
    scene = Scene.empty().with_plane(
        "mesa", Plane(point=Point(0.0, 0.0, 0.0), normal=Point(0.0, 0.0, 1.0))
    ).with_obstacle("caja", SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05))

    adapter = StaticPerceptionAdapter(scene)

    assert adapter.get_scene() is scene


def test_two_calls_return_the_same_scene():
    scene = Scene.empty()
    adapter = StaticPerceptionAdapter(scene)

    assert adapter.get_scene() == adapter.get_scene()
