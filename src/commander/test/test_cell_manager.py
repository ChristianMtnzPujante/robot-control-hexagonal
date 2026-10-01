"""Tests del gestor de células con una ejecución FALSA (sin CoppeliaSim ni
robot): crear, abrir, cerrar, el mundo que se crea y se actualiza, y lo
que `describe` le da al servidor MCP."""

import json
import math
from contextlib import contextmanager

import pytest

from commander.cell import InvalidCellError
from commander.cell.direct import CellHandle, configuration_from_degrees
from commander.cell_manager import CellManager
from commander.world import ACTION, INITIAL, SIMULATOR
from shared_kernel import GripperState, Point, Pose

from cell_fixtures import PADS, TOOL, cell


class FakeGripper:
    def __init__(self):
        self.state = GripperState(0.0, True, False, 0)
        self.held_body = None

    def get_state(self):
        return self.state


class FakeManipulator:
    """Hace como si cogiera y dejara: cambia la pinza y apunta lo pedido."""

    def __init__(self, joint_names, grabs=True):
        self.gripper = FakeGripper()
        self._configuration = configuration_from_degrees(joint_names, [0] * len(joint_names))
        self._grabs = grabs
        self.placed_at = None
        self.moved_to = None

    def move_to_position(self, point):
        self.moved_to = point

    def current_configuration(self):
        return self._configuration

    def pick(self, name, body):
        from commander.manipulation import GraspFailedError

        if not self._grabs:
            raise GraspFailedError(name)
        self.gripper.state = GripperState(0.45, True, True, 0)
        self.gripper.held_body = name
        return self.gripper.state

    def place(self, point):
        self.placed_at = point
        self.gripper.state = GripperState(0.0, True, False, 0)
        self.gripper.held_body = None
        return self.gripper.state


class FakeSimulation:
    def __init__(self, poses):
        self.poses = poses

    def body_pose(self, name):
        return self.poses[name]


class FakeRuntime:
    """Sustituye a `open_direct`: abre la célula con piezas falsas y apunta
    cuántas veces se abrió y cerró."""

    def __init__(self, grabs=True, sim_poses=None):
        self.grabs, self.sim_poses = grabs, sim_poses
        self.opened = self.closed = 0
        self.last = None

    @contextmanager
    def __call__(self, description):
        self.opened += 1
        manipulator = FakeManipulator(description.robot.model.joint_names, self.grabs)
        simulation = FakeSimulation(self.sim_poses) if self.sim_poses is not None else None
        self.last = CellHandle(description, manipulator, simulation)
        try:
            yield self.last
        finally:
            self.closed += 1


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        self.now += 1.0
        return self.now


@pytest.fixture
def grasping_cell(tree):
    return tree(tool=dict(TOOL, grasp={"offset": 0.1, "pads": PADS}))


def _manager(runtime):
    return CellManager(open_runtime=runtime, clock=Clock())


def test_create_registers_without_opening(grasping_cell):
    runtime = FakeRuntime()
    manager = _manager(runtime)
    manager.create_cell(grasping_cell)
    assert manager.list_cells() == [{"name": "prueba", "target": "sim", "open": False}]
    assert runtime.opened == 0
    info = manager.describe("prueba")
    assert info["open"] is False and info["operations"] == []
    assert "manipulation.pick_place" in [c["name"] for c in info["capabilities"]]


def test_an_invalid_cell_is_rejected_with_its_reason(tree):
    manager = _manager(FakeRuntime())
    with pytest.raises(InvalidCellError, match="kinematics"):
        manager.create_cell(tree(cell_data=cell(kinematics="dh")))


def test_opening_builds_the_world_from_the_initial_scene_and_reads_the_hardware(grasping_cell):
    manager = _manager(FakeRuntime())
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    world = manager.world("prueba")
    assert world.body_provenance("cubo").source == INITIAL  # sin vista del simulador, queda la inicial
    assert world.holding is False and world.robot_configuration is not None


def test_with_a_simulator_view_the_world_takes_its_exact_poses(grasping_cell):
    runtime = FakeRuntime(sim_poses={"cubo": Pose(0.31, 0.01, 0.025)})
    manager = _manager(runtime)
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    world = manager.world("prueba")
    assert world.scene.bodies["cubo"].pose == Pose(0.31, 0.01, 0.025)
    assert world.body_provenance("cubo").source == SIMULATOR


def test_pick_and_place_update_the_world_and_the_available_operations(grasping_cell):
    manager = _manager(FakeRuntime())
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    operations = {op["name"]: op for op in manager.describe("prueba")["operations"]}
    assert operations["pick"]["parameters"]["body"]["enum"] == ["cubo"] and "place" not in operations

    assert manager.pick("prueba", "cubo")
    world = manager.world("prueba")
    assert world.held_body == "cubo"
    operations = {op["name"]: op for op in manager.describe("prueba")["operations"]}
    assert "pick" not in operations and operations["place"]["parameters"]["point"]["enum"] == ["destino"]

    manager.place("prueba", "destino")
    assert world.held_body is None
    assert world.scene.bodies["cubo"].pose.y == pytest.approx(0.2)
    assert world.body_provenance("cubo").source == ACTION


def test_a_failed_pick_leaves_the_hands_empty(grasping_cell):
    manager = _manager(FakeRuntime(grabs=False))
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    assert not manager.pick("prueba", "cubo")
    assert manager.world("prueba").holding is False


def test_listeners_hear_lifecycle_and_world_changes(grasping_cell):
    manager = _manager(FakeRuntime())
    events = []
    manager.subscribe(events.append)
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    manager.pick("prueba", "cubo")
    manager.close_cell("prueba")
    kinds = [e.kind for e in events]
    assert kinds[:2] == ["created", "opened"] and kinds[-1] == "closed" and "world" in kinds


def test_close_releases_the_runtime_and_the_world(grasping_cell):
    runtime = FakeRuntime()
    manager = _manager(runtime)
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    manager.close_cell("prueba")
    assert runtime.closed == 1
    with pytest.raises(InvalidCellError, match="no está abierta"):
        manager.world("prueba")


def test_an_open_cell_cannot_be_replaced(grasping_cell):
    manager = _manager(FakeRuntime())
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    with pytest.raises(InvalidCellError, match="ciérrala"):
        manager.create_cell(grasping_cell)


def test_describe_is_plain_json_with_the_world_and_its_provenance(grasping_cell):
    manager = _manager(FakeRuntime())
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    info = json.loads(json.dumps(manager.describe("prueba")))
    cube = info["world"]["bodies"]["cubo"]
    assert cube["source"] == INITIAL and cube["graspable"] and cube["age_seconds"] >= 0
    assert info["world"]["robot_joints_degrees"] == {"j1": 0.0, "j2": 0.0, "j3": 0.0}


def test_unknown_cells_and_names_are_reported(grasping_cell):
    manager = _manager(FakeRuntime())
    with pytest.raises(InvalidCellError, match="Creadas: ninguna"):
        manager.open_cell("prueba")
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    with pytest.raises(InvalidCellError, match='"mesa"'):
        manager.pick("prueba", "mesa")
    with pytest.raises(InvalidCellError, match='"luna"'):
        manager.place("prueba", "luna")


def test_define_point_adds_a_destination_that_place_can_use(grasping_cell):
    manager = _manager(FakeRuntime())
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    result = manager.execute("prueba", "define_point", {"name": "caja", "x": 0.1, "y": -0.2, "z": 0.05})
    assert result["ok"]
    assert manager.world("prueba").scene.objects["caja"] == Point(0.1, -0.2, 0.05)
    manager.pick("prueba", "cubo")
    place = next(op for op in manager.describe("prueba")["operations"] if op["name"] == "place")
    assert place["parameters"]["point"]["enum"] == ["destino", "caja"]


def test_move_to_position_takes_numbers_and_rejects_anything_else(grasping_cell):
    runtime = FakeRuntime()
    manager = _manager(runtime)
    manager.create_cell(grasping_cell)
    manager.open_cell("prueba")
    assert manager.execute("prueba", "move_to_position", {"x": 0.3, "y": 0, "z": 0.2})["ok"]
    assert runtime.last.manipulator.moved_to == Point(0.3, 0.0, 0.2)
    bad = manager.execute("prueba", "move_to_position", {"x": "lejos", "y": 0, "z": 0.2})
    assert not bad["ok"] and "x:" in bad["error"] and "no es un número" in bad["error"]
    missing = manager.execute("prueba", "move_to_position", {"x": 0.3})
    assert not missing["ok"] and missing["missing"] == ["y", "z"]
