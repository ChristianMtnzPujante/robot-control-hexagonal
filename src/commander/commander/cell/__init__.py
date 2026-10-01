"""Células de trabajo descritas de forma declarativa (`scenarios/*.yaml`) y
construidas en modo directo (`direct.py`) o, más adelante, por ROS."""

from .description import CellDescription, InvalidCellError, RobotSpec, SimulatorSpec, ToolSpec
from .direct import CellHandle, open_direct
from .loader import load_cell, parse_cell, resolve_scenario

__all__ = [
    "CellDescription",
    "CellHandle",
    "InvalidCellError",
    "RobotSpec",
    "SimulatorSpec",
    "ToolSpec",
    "load_cell",
    "open_direct",
    "parse_cell",
    "resolve_scenario",
]
