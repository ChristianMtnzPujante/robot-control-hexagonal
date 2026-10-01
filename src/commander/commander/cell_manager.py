"""El núcleo de `Commander` (ver "Roles del Commander" en el vault): gestiona
células -- crearlas válidas, abrirlas, cerrarlas -- y lleva el MUNDO de
cada una. Es lo que consultará el servidor MCP para saber qué tools
ofrecer (`describe`) y cuándo volver a preguntar (`subscribe`).

No sabe de ROS ni de CoppeliaSim: abre las células con `open_runtime`, que
hoy es el modo directo (`cell.open_direct`) y en la fase 2 será también el
de sesiones ROS -- dos implementaciones del mismo puerto de salida. El
nodo ROS `Commander` (`commander_node.py`) le delegará cuando llegue esa
fase.

`pick` y `place` son las primeras habilidades: existen sobre todo para que
el mundo se entere de sus efectos. El resto (y el guardián) llegan después.
"""

from __future__ import annotations

import math
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

from shared_kernel import Point, Pose

from .cell import CellDescription, InvalidCellError, compile_cell, open_direct, parse_cell
from .cell import capabilities as caps
from .cell.adapters import cell_capabilities
from .world import ACTION, HARDWARE, SIMULATOR, World


@dataclass(frozen=True)
class CellEvent:
    """`kind`: "created", "opened", "closed" o "world" (cambió su mundo)."""

    kind: str
    cell: str


@dataclass
class _Cell:
    description: CellDescription
    capabilities: frozenset
    stack: Optional[ExitStack] = None
    handle: Any = None
    world: Optional[World] = None

    @property
    def is_open(self) -> bool:
        return self.stack is not None


class CellManager:
    def __init__(
        self,
        open_runtime: Callable = open_direct,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._open_runtime = open_runtime
        self._clock = clock
        self._cells: Dict[str, _Cell] = {}
        self._listeners: List[Callable[[CellEvent], None]] = []

    # --- Gestión de células ----------------------------------------------------------

    def subscribe(self, listener: Callable[[CellEvent], None]) -> None:
        self._listeners.append(listener)

    def create_cell(self, source: Union[str, Path, dict, CellDescription]) -> CellDescription:
        """Compila y registra una célula SIN abrirla. `source`: un nombre de
        `descriptions/cells/`, una ruta, el contenido de un YAML ya leído
        (dict) o una `CellDescription`. Si no es válida, lanza
        `InvalidCellError` con el sitio del error -- es lo que verá quien la
        haya escrito (una persona o el LLM)."""
        if isinstance(source, CellDescription):
            description = source
        elif isinstance(source, dict):
            description = parse_cell(source)
        else:
            description = compile_cell(source)
        existing = self._cells.get(description.name)
        if existing is not None and existing.is_open:
            raise InvalidCellError(f'la célula "{description.name}" está abierta: ciérrala antes de sustituirla')
        self._cells[description.name] = _Cell(description, cell_capabilities(description))
        self._emit("created", description.name)
        return description

    def list_cells(self) -> List[Dict[str, Any]]:
        return [
            {"name": name, "target": cell.description.robot.target, "open": cell.is_open}
            for name, cell in self._cells.items()
        ]

    def open_cell(self, name: str) -> None:
        cell = self._get(name)
        if cell.is_open:
            return
        stack = ExitStack()
        try:
            cell.handle = stack.enter_context(self._open_runtime(cell.description))
        except BaseException:
            stack.close()
            raise
        cell.stack = stack
        cell.world = World(cell.description.scene, clock=self._clock)
        cell.world.subscribe(lambda change, n=name: self._emit("world", n))
        self._emit("opened", name)
        self.refresh_world(name)

    def close_cell(self, name: str) -> None:
        cell = self._get(name)
        if not cell.is_open:
            return
        try:
            cell.stack.close()
        finally:
            cell.stack, cell.handle, cell.world = None, None, None
            self._emit("closed", name)

    def close_all(self) -> None:
        for name in list(self._cells):
            self.close_cell(name)

    def handle(self, name: str):
        """El `CellHandle` de una célula abierta (su `Manipulator`, sus
        posturas...), para scripts que actúan directamente."""
        return self._open(name).handle

    def world(self, name: str) -> World:
        return self._open(name).world

    # --- Mundo -------------------------------------------------------------------

    def refresh_world(self, name: str) -> None:
        """Lee lo que se pueda leer ahora: la configuración del brazo, el
        estado de la pinza y, en simulación, la pose exacta de cada cuerpo."""
        cell = self._open(name)
        manipulator, world = cell.handle.manipulator, cell.world
        world.set_robot(manipulator.current_configuration(), SIMULATOR if self._is_sim(cell) else HARDWARE)
        if manipulator.gripper is not None:
            held = getattr(manipulator.gripper, "held_body", None)  # solo la simulada sabe el nombre
            world.set_gripper(manipulator.gripper.get_state(), SIMULATOR if self._is_sim(cell) else HARDWARE, held)
        if caps.SIM_GROUND_TRUTH in cell.capabilities and cell.handle.simulation is not None:
            for body in cell.description.scene.bodies:
                world.move_body(body, cell.handle.simulation.body_pose(body), SIMULATOR, self._clock())

    def pick(self, name: str, body: str) -> bool:
        """Coge `body` y anota en el mundo que lo sujeta. Devuelve si lo
        cogió (si no, `Manipulator` ya ha abierto y se ha retirado)."""
        cell = self._open(name)
        world = cell.world
        if body not in world.scene.bodies:
            raise InvalidCellError(f'no hay ningún cuerpo "{body}" en el mundo de "{name}"')
        from .manipulation import GraspFailedError

        try:
            state = cell.handle.manipulator.pick(body, world.scene.bodies[body])
        except GraspFailedError:
            world.set_gripper(cell.handle.manipulator.gripper.get_state(), ACTION)
            return False
        world.set_gripper(state, ACTION, held_body=body)
        return True

    def place(self, name: str, point: str) -> None:
        """Deja lo que se sujeta en el punto con nombre `point` y anota en
        el mundo dónde queda (lo esperado; una lectura posterior del
        simulador o de percepción lo corrige si hace falta)."""
        cell = self._open(name)
        world = cell.world
        if point not in world.scene.objects:
            raise InvalidCellError(f'no hay ningún punto "{point}" en el mundo de "{name}"')
        held = world.held_body
        target: Point = world.scene.objects[point]
        state = cell.handle.manipulator.place(target)
        if held is not None:
            current = world.scene.bodies[held].pose
            world.move_body(held, Pose(target.x, target.y, target.z, current.qx, current.qy, current.qz, current.qw),
                            ACTION)
        world.set_gripper(state, ACTION)

    # --- Para el servidor MCP ------------------------------------------------------

    def describe(self, name: str) -> Dict[str, Any]:
        """Todo lo que un cliente (el servidor MCP) necesita para decidir
        qué tools ofrecer y con qué opciones: capacidades fijas de la
        célula, operaciones disponibles AHORA con sus opciones, y el mundo
        con el origen y la antigüedad de cada dato. Solo tipos JSON."""
        cell = self._get(name)
        description = cell.description
        info: Dict[str, Any] = {
            "name": name,
            "target": description.robot.target,
            "robot": description.robot.model.name,
            "tool": description.tool.model.name if description.tool else None,
            "open": cell.is_open,
            "capabilities": [
                {"name": capability, "description": caps.DESCRIPTIONS.get(capability, "")}
                for capability in sorted(cell.capabilities)
            ],
        }
        if not cell.is_open:
            info["operations"] = []
            return info
        world = cell.world
        operations = caps.available_operations(
            cell.capabilities,
            postures=description.postures,
            points=world.scene.objects,
            graspable_bodies=world.scene.graspable_bodies(),
            holding=world.holding,
            held_body=world.held_body,
        )
        info["operations"] = [
            {"name": op.name, "requires": op.requires, "description": op.description,
             "options": {parameter: list(values) for parameter, values in op.options}}
            for op in operations
        ]
        info["world"] = self._world_summary(world)
        return info

    def _world_summary(self, world: World) -> Dict[str, Any]:
        now = self._clock()
        bodies = {}
        for body_name, body in world.scene.bodies.items():
            provenance = world.body_provenance(body_name)
            p = body.pose
            bodies[body_name] = {
                "shape": type(body.shape).__name__.lower(),
                "position": [round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                "graspable": body.graspable,
                "source": provenance.source,
                "age_seconds": round(now - provenance.timestamp, 3),
            }
        configuration = world.robot_configuration
        return {
            "bodies": bodies,
            "points": {n: [p.x, p.y, p.z] for n, p in world.scene.objects.items()},
            "holding": world.holding,
            "held_body": world.held_body,
            "robot_joints_degrees": (
                {j.joint_name: round(math.degrees(j.angle_radians), 2) for j in configuration.positions}
                if configuration is not None else None
            ),
        }

    # --- Internos ----------------------------------------------------------------

    def _get(self, name: str) -> _Cell:
        if name not in self._cells:
            available = ", ".join(self._cells) or "ninguna"
            raise InvalidCellError(f'no hay ninguna célula "{name}". Creadas: {available}')
        return self._cells[name]

    def _open(self, name: str) -> _Cell:
        cell = self._get(name)
        if not cell.is_open:
            raise InvalidCellError(f'la célula "{name}" no está abierta')
        return cell

    @staticmethod
    def _is_sim(cell: _Cell) -> bool:
        return cell.description.robot.target == "sim"

    def _emit(self, kind: str, name: str) -> None:
        event = CellEvent(kind, name)
        for listener in list(self._listeners):
            listener(event)
