"""Tests del modelo del mundo: la regla de confianza entre fuentes, el
origen de cada dato y los avisos de cambio."""

import pytest

from commander.world import ACTION, INITIAL, PERCEPTION, SIMULATOR, Provenance, World, accepts
from shared_kernel import Body, Box, GripperState, Pose, Scene

_CUBE = Body(Box(0.05, 0.05, 0.05), Pose(0.3, 0, 0.025), graspable=True)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _world():
    clock = Clock()
    return World(Scene.empty().with_body("cubo", _CUBE), clock=clock), clock


def _at(x):
    return Pose(x, 0, 0.025)


@pytest.mark.parametrize(
    "current, new, accepted",
    [
        ((INITIAL, 0), (ACTION, 1), True),
        ((PERCEPTION, 1), (SIMULATOR, 2), True),
        ((SIMULATOR, 1), (PERCEPTION, 2), False),  # en sim, la percepción no pisa la verdad
        ((PERCEPTION, 1), (ACTION, 2), True),  # lo que hacemos invalida lo visto antes
        ((SIMULATOR, 1), (ACTION, 2), True),
        ((ACTION, 1), (PERCEPTION, 2), True),  # y lo visto después corrige lo esperado
        ((PERCEPTION, 2), (PERCEPTION, 1), False),  # nunca algo más antiguo
        ((PERCEPTION, 2), (SIMULATOR, 1), False),
        ((ACTION, 2), (ACTION, 2), True),
    ],
)
def test_trust_rule(current, new, accepted):
    assert accepts(Provenance(*current), Provenance(*new)) is accepted


def test_a_new_world_starts_from_the_initial_scene():
    world, _ = _world()
    assert world.scene.bodies["cubo"] == _CUBE
    assert world.body_provenance("cubo").source == INITIAL
    assert world.holding is None and world.held_body is None and world.robot_configuration is None


def test_moving_a_body_keeps_its_shape_and_records_where_it_came_from():
    world, clock = _world()
    clock.now = 5.0
    assert world.move_body("cubo", _at(0.1), PERCEPTION)
    assert world.scene.bodies["cubo"].pose == _at(0.1)
    assert world.scene.bodies["cubo"].shape == _CUBE.shape
    assert world.body_provenance("cubo") == Provenance(PERCEPTION, 5.0)


def test_a_rejected_update_changes_nothing_and_notifies_nobody():
    world, clock = _world()
    clock.now = 1.0
    world.move_body("cubo", _at(0.1), SIMULATOR)
    changes = []
    world.subscribe(changes.append)
    clock.now = 2.0
    assert not world.move_body("cubo", _at(0.2), PERCEPTION)
    assert world.scene.bodies["cubo"].pose == _at(0.1) and changes == []


def test_moving_an_unknown_body_is_an_error():
    world, _ = _world()
    with pytest.raises(KeyError):
        world.move_body("fantasma", _at(0), PERCEPTION)


def test_perception_can_add_a_new_body():
    world, _ = _world()
    taza = Body(Box(0.08, 0.08, 0.1), Pose(0.1, 0.1, 0.05), graspable=True)
    assert world.observe_body("taza", taza, PERCEPTION)
    assert world.scene.bodies["taza"] == taza and world.body_provenance("taza").source == PERCEPTION


def test_gripper_state_tracks_what_is_held_and_forgets_it_when_released():
    world, _ = _world()
    world.set_gripper(GripperState(0.45, True, True, 0), ACTION, held_body="cubo")
    assert world.holding and world.held_body == "cubo"
    world.set_gripper(GripperState(0.0, True, False, 0), ACTION)
    assert world.holding is False and world.held_body is None


def test_listeners_hear_every_accepted_change():
    world, clock = _world()
    changes = []
    world.subscribe(changes.append)
    clock.now = 1.0
    world.move_body("cubo", _at(0.1), ACTION)
    world.set_gripper(GripperState(0.0, True, False, 0), ACTION)
    assert [(c.kind, c.name) for c in changes] == [("body", "cubo"), ("gripper", None)]
