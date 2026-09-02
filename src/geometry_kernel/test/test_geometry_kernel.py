import pytest

from geometry_kernel import Plane, Point, Scene, SphereObstacle


def test_scene_empty_has_nothing():
    scene = Scene.empty()
    assert scene.obstacles == {}
    assert scene.planes == {}
    assert scene.objects == {}


def test_with_obstacle_does_not_mutate_original():
    scene = Scene.empty()
    obstacle = SphereObstacle(center=Point(0, 0, 0), radius=0.1)

    updated = scene.with_obstacle("caja", obstacle)

    assert scene.obstacles == {}
    assert updated.obstacles == {"caja": obstacle}


def test_with_obstacle_overwrites_by_name_instead_of_duplicating():
    obstacle = SphereObstacle(center=Point(0, 0, 0), radius=0.1)
    moved = SphereObstacle(center=Point(1, 0, 0), radius=0.1)

    scene = Scene.empty().with_obstacle("caja", obstacle).with_obstacle("caja", moved)

    assert scene.obstacles == {"caja": moved}


def test_merge_combines_two_scenes_and_other_wins_on_key_conflicts():
    plane = Plane(point=Point(0, 0, 0), normal=Point(0, 0, 1))
    obstacle = SphereObstacle(center=Point(0, 0, 0), radius=0.1)
    moved = SphereObstacle(center=Point(1, 0, 0), radius=0.1)

    a = Scene.empty().with_plane("mesa", plane).with_obstacle("caja", obstacle)
    b = Scene.empty().with_obstacle("caja", moved).with_object("taza", Point(0, 0, 0))

    merged = a.merge(b)

    assert merged.planes == {"mesa": plane}
    assert merged.obstacles == {"caja": moved}
    assert merged.objects == {"taza": Point(0, 0, 0)}


def test_sphere_obstacle_rejects_non_positive_radius():
    with pytest.raises(ValueError):
        SphereObstacle(center=Point(0, 0, 0), radius=0)


def test_with_plane_and_with_object():
    scene = Scene.empty()
    plane = Plane(point=Point(0, 0, 0), normal=Point(0, 0, 1))

    scene = scene.with_plane("mesa", plane)
    scene = scene.with_object("taza", Point(0.2, 0.1, 0.05))

    assert scene.planes["mesa"] == plane
    assert scene.objects["taza"] == Point(0.2, 0.1, 0.05)
