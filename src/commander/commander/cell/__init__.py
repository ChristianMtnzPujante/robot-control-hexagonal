"""Células de trabajo descritas por piezas en YAML (`descriptions/`: robots,
herramientas, escenas y células que las componen), compiladas en una
`CellDescription` y construidas en modo directo (`direct.py`) o, más
adelante, por ROS. Guía de formatos: `guide.py`."""

from .compile import compile_cell, parse_cell, resolve_cell
from .description import CellDescription, RobotSpec, SimulatorSpec, ToolSpec
from .direct import CellHandle, open_direct
from .elements import RobotModel, ToolModel, load_robot, load_tool
from .errors import InvalidCellError

__all__ = [
    "CellDescription",
    "CellHandle",
    "InvalidCellError",
    "RobotModel",
    "RobotSpec",
    "SimulatorSpec",
    "ToolModel",
    "ToolSpec",
    "compile_cell",
    "load_robot",
    "load_tool",
    "open_direct",
    "parse_cell",
    "resolve_cell",
]
