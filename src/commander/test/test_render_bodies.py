"""Tests de `_render_bodies` contra un `sim` de mentira: qué shapes pide a
CoppeliaSim para cada cuerpo de la `Scene`. NO sustituyen verlo en
CoppeliaSim."""

import pytest

from commander.coppeliasim_scene_builder import _render_bodies
from shared_kernel import Body, Box, Cylinder, Pose, Scene, Sphere


class FakeSim:
    primitiveshape_cuboid = "cuboid"
    primitiveshape_cylinder = "cylinder"
    primitiveshape_spheroid = "spheroid"
    colorcomponent_ambient_diffuse = "diffuse"
    shapeintparam_static = "static"
    shapeintparam_respondable = "respondable"

    def __init__(self):
        self.objects = {}
        self._next = 0

    def _new(self, **fields):
        self._next += 1
        self.objects[self._next] = dict(fields, params={})
        return self._next

    def createDummy(self, size):
        return self._new(kind="dummy")

    def createPrimitiveShape(self, kind, sizes):
        return self._new(kind=kind, sizes=list(sizes))

    def setObjectAlias(self, handle, alias):
        self.objects[handle]["alias"] = alias

    def setShapeColor(self, handle, name, component, rgb):
        self.objects[handle]["color"] = list(rgb)

    def setObjectInt32Param(self, handle, param, value):
        self.objects[handle]["params"][param] = value

    def setObjectPose(self, handle, relative_to, pose):
        self.objects[handle]["pose"] = list(pose)

    def setObjectParent(self, handle, parent, keep_in_place):
        self.objects[handle]["parent"] = parent

    def by_alias(self, alias):
        return next(o for o in self.objects.values() if o.get("alias") == alias)


def _render(scene):
    sim = FakeSim()
    _render_bodies(sim, scene)
    return sim


def test_an_empty_scene_creates_nothing():
    assert _render(Scene.empty()).objects == {}


def test_each_shape_gets_its_coppeliasim_sizes():
    sim = _render(
        Scene.empty()
        .with_body("mesa", Body(Box(1.0, 0.8, 0.05), Pose(0, 0, 0)))
        .with_body("lata", Body(Cylinder(0.03, 0.1), Pose(0, 0, 0)))
        .with_body("bola", Body(Sphere(0.02), Pose(0, 0, 0)))
    )
    assert sim.by_alias("mesa")["kind"] == "cuboid"
    assert sim.by_alias("mesa")["sizes"] == [1.0, 0.8, 0.05]
    assert sim.by_alias("lata")["sizes"] == pytest.approx([0.06, 0.06, 0.1])
    assert sim.by_alias("bola")["sizes"] == pytest.approx([0.04, 0.04, 0.04])


def test_bodies_are_static_posed_and_hang_from_one_root():
    pose = Pose(-0.5, 0.1, 0.025, 0.0, 0.0, 0.7071, 0.7071)
    sim = _render(Scene.empty().with_body("cubo", Body(Box(0.05, 0.05, 0.05), pose)))
    cube = sim.by_alias("cubo")
    root = next(h for h, o in sim.objects.items() if o.get("alias") == "cuerpos_escena")
    assert cube["pose"] == [-0.5, 0.1, 0.025, 0.0, 0.0, 0.7071, 0.7071]
    assert cube["params"] == {"static": 1, "respondable": 0}
    assert cube["parent"] == root


def test_color_follows_graspable_unless_given():
    sim = _render(
        Scene.empty()
        .with_body("cubo", Body(Box(0.05, 0.05, 0.05), Pose(0, 0, 0), graspable=True))
        .with_body("mesa", Body(Box(1, 1, 0.05), Pose(0, 0, 0)))
        .with_body("azul", Body(Sphere(0.02), Pose(0, 0, 0), color=(0.0, 0.0, 1.0)))
    )
    assert sim.by_alias("cubo")["color"] != sim.by_alias("mesa")["color"]
    assert sim.by_alias("azul")["color"] == [0.0, 0.0, 1.0]
