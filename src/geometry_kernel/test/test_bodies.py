import math

import pytest

from geometry_kernel import Body, Box, Cylinder, Point, Pose, Scene, Sphere

_CUBE = Body(Box(0.05, 0.05, 0.05), Pose(-0.5, 0.0, 0.025), graspable=True)
_TABLE = Body(Box(1.0, 1.0, 0.05), Pose(-0.3, 0.0, -0.025))


@pytest.mark.parametrize(
    "shape", [lambda: Box(0.1, 0.0, 0.1), lambda: Cylinder(-0.1, 0.2), lambda: Sphere(0)]
)
def test_shapes_reject_non_positive_sizes(shape):
    with pytest.raises(ValueError):
        shape()


def test_bounding_sphere_is_centered_on_the_body_and_contains_the_shape():
    sphere = _CUBE.bounding_sphere()
    assert sphere.center == Point(-0.5, 0.0, 0.025)
    assert sphere.radius == pytest.approx(0.05 * math.sqrt(3) / 2)


def test_bounding_radius_of_a_cylinder_reaches_the_rim():
    assert Cylinder(0.03, 0.08).bounding_radius() == pytest.approx(0.05)


def test_with_body_keeps_the_rest_of_the_scene():
    scene = Scene.empty().with_object("objetivo", Point(0, 0, 1)).with_body("cubo", _CUBE)
    assert scene.bodies == {"cubo": _CUBE}
    assert scene.objects == {"objetivo": Point(0, 0, 1)}


def test_graspable_bodies_leaves_out_fixed_ones():
    scene = Scene.empty().with_body("mesa", _TABLE).with_body("cubo", _CUBE)
    assert scene.graspable_bodies() == {"cubo": _CUBE}


def test_merge_includes_bodies_and_other_wins():
    moved = Body(Box(0.05, 0.05, 0.05), Pose(-0.4, 0.1, 0.025), graspable=True)
    merged = (
        Scene.empty().with_body("mesa", _TABLE).with_body("cubo", _CUBE)
        .merge(Scene.empty().with_body("cubo", moved))
    )
    assert merged.bodies == {"mesa": _TABLE, "cubo": moved}
