"""Compila una CÉLULA (`descriptions/cells/*.yaml`): resuelve sus
referencias al robot, la herramienta y la escena (cada uno en su propio
formato y fichero), valida cada pieza con su esquema, y devuelve la
`CellDescription` resuelta, que valida a su vez las reglas entre piezas.

El fichero de la célula solo dice qué piezas se usan y cómo:

    name: mesa_cubo
    robot: {ref: cr5, target: sim, initial_posture: home}
    tools: [{ref: robotiq_2f_85}]
    scene: {ref: mesa_cubo}
    kinematics: poe
    postures: {pre_agarre: [0, 20, 100, -30, -90, 0]}

Un `ref` es un nombre de `descriptions/<tipo>/` o una ruta a un .yaml
relativa a la célula. La guía completa de campos la genera `guide.py`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Union

from .adapters import REAL_GRIPPERS, REAL_ROBOTS, SIM_GRIPPERS
from .description import CellDescription, RobotSpec, SimulatorSpec, ToolSpec
from .elements import DESCRIPTIONS_DIR, load_robot, load_tool, read_yaml, resolve_ref
from .errors import InvalidCellError
from .scene_format import parse_scene
from .schema import Field, Section, number, numbers, mapping, text

CELL = Section(
    "Célula",
    "descriptions/cells/<nombre>.yaml",
    (
        Field("name", "texto", True, "Nombre de la célula."),
        Field("robot", "sección", True, "Qué robot y contra qué destino (ver «Célula: robot»)."),
        Field("tools", "lista", False, "Herramientas montadas (ver «Célula: herramienta»). De momento, una.", "[]"),
        Field("scene", "sección", False, "Escena inicial (ver «Célula: escena»). Sin ella, vacía."),
        Field("kinematics", "`poe` | `ga`", False, "Cinemática, construida del URDF del robot.", "poe"),
        Field("simulator", "sección", False, "CoppeliaSim (ver «Célula: simulador»)."),
        Field("postures", "dict nombre → grados", False, "Posturas de esta tarea (p. ej. `pre_agarre`). Se suman a las del robot; no pueden repetir nombre."),
    ),
)

CELL_ROBOT = Section("Célula: robot", "cell: robot", (
    Field("ref", "nombre o ruta", True, "Robot de `descriptions/robots/`."),
    Field("target", "`sim` | `real`", False, "Dónde está el robot. `--target` lo cambia al lanzar.", "sim"),
    Field("host", "IP", "si `real`", "IP del controlador."),
    Field("initial_posture", "nombre", False, "Postura en la que se crea en simulación. En real no se mueve nada al abrir.", "todo a 0"),
    Field("speed_factor", "entero 1-100", False, "Solo real: velocidad global en % (`SpeedFactor`). Sin él, la del controlador."),
))

CELL_TOOL = Section("Célula: herramienta", "cell: tools[i]", (
    Field("ref", "nombre o ruta", True, "Herramienta de `descriptions/tools/`."),
    Field("grasp_offset", "m", "si `real`", "Distancia de agarre MEDIDA en este montaje. Sin ella, la del modelo (solo simulación)."),
))

CELL_SCENE = Section("Célula: escena", "cell: scene", (
    Field("ref", "nombre o ruta", True, "Escena de `descriptions/scenes/`."),
))

CELL_SIMULATOR = Section("Célula: simulador", "cell: simulator", (
    Field("port", "entero", False, "Puerto ZMQ de CoppeliaSim.", "23000"),
    Field("step_pause_seconds", "s", False, "Pausa entre waypoints (para ver la animación).", "0.04"),
))


def resolve_cell(name_or_path: Union[str, Path]) -> Path:
    """Un fichero que existe, o un nombre de `descriptions/cells/`."""
    if Path(name_or_path).expanduser().is_file():
        return Path(name_or_path).expanduser().resolve()
    return resolve_ref("cells", str(name_or_path))


def compile_cell(name_or_path: Union[str, Path]) -> CellDescription:
    path = resolve_cell(name_or_path)
    try:
        return parse_cell(read_yaml(path), path.parent)
    except InvalidCellError as error:
        raise InvalidCellError(f"{path.name}: {error}") from error


def parse_cell(raw: Any, base_dir: Path = DESCRIPTIONS_DIR / "cells") -> CellDescription:
    data = CELL.check(raw, "célula")

    robot_data = CELL_ROBOT.check(data["robot"], "robot")
    robot_model = load_robot(text(robot_data["ref"], "robot.ref"), base_dir)
    _check_adapter(robot_model.real_adapter, REAL_ROBOTS, f"robots/{robot_model.name}.real.adapter")

    tools = []
    for i, raw_tool in enumerate(data.get("tools") or []):
        tool_data = CELL_TOOL.check(raw_tool, f"tools[{i}]")
        tool_model = load_tool(text(tool_data["ref"], f"tools[{i}].ref"), base_dir)
        _check_adapter(tool_model.sim_adapter, SIM_GRIPPERS, f"tools/{tool_model.name}.sim.adapter")
        _check_adapter(tool_model.real_adapter, REAL_GRIPPERS, f"tools/{tool_model.name}.real.adapter")
        offset = tool_data.get("grasp_offset")
        tools.append(ToolSpec(tool_model, None if offset is None else number(offset, f"tools[{i}].grasp_offset")))

    scene_data = data.get("scene")
    if scene_data is None:
        scene = parse_scene({}, "scene")
    else:
        scene_ref = text(CELL_SCENE.check(scene_data, "scene")["ref"], "scene.ref")
        scene_path = resolve_ref("scenes", scene_ref, base_dir)
        scene = parse_scene(read_yaml(scene_path), f"scenes/{scene_path.stem}")

    simulator_data = CELL_SIMULATOR.check(data.get("simulator") or {}, "simulator")
    defaults = SimulatorSpec()

    postures = dict(robot_model.postures)
    for name, values in mapping(data.get("postures") or {}, "postures").items():
        if name in postures:
            raise InvalidCellError(f'la postura "{name}" ya la define el robot "{robot_model.name}"')
        postures[name] = numbers(values, f"postures.{name}")

    return CellDescription(
        name=text(data["name"], "name"),
        robot=RobotSpec(
            model=robot_model,
            target=robot_data.get("target", "sim"),
            host=robot_data.get("host"),
            initial_posture=robot_data.get("initial_posture"),
            speed_factor=None if robot_data.get("speed_factor") is None
            else int(number(robot_data["speed_factor"], "robot.speed_factor")),
        ),
        tools=tuple(tools),
        kinematics=data.get("kinematics", "poe"),
        simulator=SimulatorSpec(
            port=int(simulator_data.get("port", defaults.port)),
            step_pause_seconds=number(
                simulator_data.get("step_pause_seconds", defaults.step_pause_seconds), "simulator.step_pause_seconds"
            ),
        ),
        postures=postures,
        scene=scene,
    )


def _check_adapter(name, registry, where: str) -> None:
    if name is not None and name not in registry:
        raise InvalidCellError(f'{where}: adaptador "{name}" desconocido. Hay: {", ".join(sorted(registry))}')
