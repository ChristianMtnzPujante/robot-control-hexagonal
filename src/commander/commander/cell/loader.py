"""Lee una `CellDescription` de YAML (`scenarios/*.yaml`).

Estricto a propósito: una clave desconocida es un error que dice dónde
está, no algo que se ignora -- un `graspabel: true` mal escrito dejaría un
cubo sin poder cogerse sin que nada avisara. Formato (todo en metros y
grados):

    name: mesa_cubo
    robot:
      model: cr5
      target: sim            # sim | real
      host: 192.168.5.1      # solo con target real
      initial_posture: home  # opcional, solo simulación
    tools:
      - model: robotiq_2f_85
        grasp_offset: 0.14   # opcional en sim; obligatorio (medido) en real
    kinematics: poe          # poe | ga
    simulator:
      port: 23000
      step_pause_seconds: 0.04
    postures:                # grados, en el orden de joints del robot
      home: [0, 0, 0, 0, 0, 0]
    scene:
      bodies:
        cubo:
          shape: box         # box | cylinder | sphere
          size: [0.05, 0.05, 0.05]   # box: medidas totales x, y, z
          # cylinder: radius + height (eje en z); sphere: radius
          position: [-0.571, -0.141, 0.025]   # centro de la forma
          rpy_degrees: [0, 0, 0]     # o quaternion: [qx, qy, qz, qw]
          graspable: true
          color: [0.85, 0.2, 0.15]   # opcional
      points:                # Scene.objects: puntos con nombre
        destino: [-0.528, 0.259, 0.025]
      obstacles:             # Scene.obstacles: esferas a evitar
        poste: {center: [0.3, 0.0, 0.4], radius: 0.05}
      planes:
        suelo: {point: [0, 0, 0], normal: [0, 0, 1]}
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple, Union

import yaml

from shared_kernel import Body, Box, Cylinder, Plane, Point, Pose, Scene, Sphere, SphereObstacle

from .description import CellDescription, InvalidCellError, RobotSpec, SimulatorSpec, ToolSpec

# scenarios/ vive en la raíz del repo, igual que assets/ (ver
# coppeliasim_scene_builder._ASSETS_DIR sobre resolve() y symlink-install).
SCENARIOS_DIR = Path(__file__).resolve().parents[4] / "scenarios"


def resolve_scenario(name_or_path: Union[str, Path]) -> Path:
    """Un fichero que existe, o el nombre de uno de `scenarios/` (con o sin
    `.yaml`)."""
    path = Path(name_or_path)
    if path.is_file():
        return path
    candidate = SCENARIOS_DIR / (path.name if path.suffix else f"{path.name}.yaml")
    if candidate.is_file():
        return candidate
    available = ", ".join(sorted(p.stem for p in SCENARIOS_DIR.glob("*.yaml")))
    raise InvalidCellError(f'no encuentro el escenario "{name_or_path}". En scenarios/: {available}')


def load_cell(name_or_path: Union[str, Path]) -> CellDescription:
    path = resolve_scenario(name_or_path)
    with open(path) as file:
        raw = yaml.safe_load(file)
    try:
        return parse_cell(raw)
    except InvalidCellError as error:
        raise InvalidCellError(f"{path}: {error}") from error


def parse_cell(raw: Any) -> CellDescription:
    data = _mapping(raw, "la raíz del fichero")
    _only(data, "la raíz", {"name", "robot", "tools", "kinematics", "simulator", "postures", "scene"})
    return CellDescription(
        name=_required(data, "name", "la raíz"),
        robot=_robot(_mapping(_required(data, "robot", "la raíz"), "robot")),
        tools=tuple(_tool(_mapping(t, f"tools[{i}]"), i) for i, t in enumerate(data.get("tools") or [])),
        kinematics=data.get("kinematics", "poe"),
        simulator=_simulator(_mapping(data.get("simulator") or {}, "simulator")),
        postures={
            name: _numbers(values, f"postures.{name}")
            for name, values in _mapping(data.get("postures") or {}, "postures").items()
        },
        scene=_scene(_mapping(data.get("scene") or {}, "scene")),
    )


# --- Secciones -----------------------------------------------------------------


def _robot(data: Dict[str, Any]) -> RobotSpec:
    _only(data, "robot", {"model", "target", "host", "initial_posture"})
    return RobotSpec(
        model=_required(data, "model", "robot"),
        target=data.get("target", "sim"),
        host=data.get("host"),
        initial_posture=data.get("initial_posture"),
    )


def _tool(data: Dict[str, Any], index: int) -> ToolSpec:
    where = f"tools[{index}]"
    _only(data, where, {"model", "grasp_offset"})
    offset = data.get("grasp_offset")
    return ToolSpec(
        model=_required(data, "model", where),
        grasp_offset=None if offset is None else _number(offset, f"{where}.grasp_offset"),
    )


def _simulator(data: Dict[str, Any]) -> SimulatorSpec:
    _only(data, "simulator", {"port", "step_pause_seconds"})
    defaults = SimulatorSpec()
    return SimulatorSpec(
        port=int(data.get("port", defaults.port)),
        step_pause_seconds=_number(
            data.get("step_pause_seconds", defaults.step_pause_seconds), "simulator.step_pause_seconds"
        ),
    )


def _scene(data: Dict[str, Any]) -> Scene:
    _only(data, "scene", {"bodies", "points", "obstacles", "planes"})
    scene = Scene.empty()
    for name, body in _mapping(data.get("bodies") or {}, "scene.bodies").items():
        scene = scene.with_body(name, _body(_mapping(body, f"scene.bodies.{name}"), f"scene.bodies.{name}"))
    for name, point in _mapping(data.get("points") or {}, "scene.points").items():
        scene = scene.with_object(name, _point(point, f"scene.points.{name}"))
    for name, obstacle in _mapping(data.get("obstacles") or {}, "scene.obstacles").items():
        where = f"scene.obstacles.{name}"
        obstacle = _mapping(obstacle, where)
        _only(obstacle, where, {"center", "radius"})
        scene = scene.with_obstacle(
            name,
            _build(
                where,
                lambda: SphereObstacle(
                    _point(_required(obstacle, "center", where), f"{where}.center"),
                    _number(_required(obstacle, "radius", where), f"{where}.radius"),
                ),
            ),
        )
    for name, plane in _mapping(data.get("planes") or {}, "scene.planes").items():
        where = f"scene.planes.{name}"
        plane = _mapping(plane, where)
        _only(plane, where, {"point", "normal"})
        scene = scene.with_plane(
            name,
            Plane(
                _point(_required(plane, "point", where), f"{where}.point"),
                _point(_required(plane, "normal", where), f"{where}.normal"),
            ),
        )
    return scene


_SHAPE_KEYS = {"box": {"size"}, "cylinder": {"radius", "height"}, "sphere": {"radius"}}
_BODY_KEYS = {"shape", "position", "rpy_degrees", "quaternion", "graspable", "color"}


def _body(data: Dict[str, Any], where: str) -> Body:
    shape_name = _required(data, "shape", where)
    if shape_name not in _SHAPE_KEYS:
        raise InvalidCellError(f'{where}.shape "{shape_name}" no es válida: {", ".join(_SHAPE_KEYS)}')
    _only(data, where, _BODY_KEYS | _SHAPE_KEYS[shape_name])
    if "rpy_degrees" in data and "quaternion" in data:
        raise InvalidCellError(f"{where}: rpy_degrees o quaternion, no los dos")

    def shape():
        if shape_name == "box":
            size = _numbers(_required(data, "size", where), f"{where}.size", length=3)
            return Box(*size)
        if shape_name == "cylinder":
            return Cylinder(
                _number(_required(data, "radius", where), f"{where}.radius"),
                _number(_required(data, "height", where), f"{where}.height"),
            )
        return Sphere(_number(_required(data, "radius", where), f"{where}.radius"))

    x, y, z = _numbers(_required(data, "position", where), f"{where}.position", length=3)
    if "quaternion" in data:
        qx, qy, qz, qw = _numbers(data["quaternion"], f"{where}.quaternion", length=4)
    else:
        rpy = _numbers(data.get("rpy_degrees", [0, 0, 0]), f"{where}.rpy_degrees", length=3)
        qx, qy, qz, qw = rpy_degrees_to_quaternion(*rpy)
    color = data.get("color")
    return Body(
        shape=_build(where, shape),
        pose=Pose(x, y, z, qx, qy, qz, qw),
        graspable=_boolean(data.get("graspable", False), f"{where}.graspable"),
        color=None if color is None else _numbers(color, f"{where}.color", length=3),
    )


def rpy_degrees_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convención URDF: giros sobre los ejes FIJOS x, y, z, en ese orden
    (R = Rz(yaw)·Ry(pitch)·Rx(roll)). Devuelve (qx, qy, qz, qw)."""
    r, p, y = (math.radians(v) / 2 for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# --- Utilidades de validación --------------------------------------------------


def _mapping(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise InvalidCellError(f"{where} debería ser un diccionario (clave: valor)")
    return value


def _only(data: Dict[str, Any], where: str, allowed: Iterable[str]) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise InvalidCellError(
            f'{where}: clave(s) desconocida(s) {", ".join(unknown)}. Válidas: {", ".join(sorted(allowed))}'
        )


def _required(data: Dict[str, Any], key: str, where: str) -> Any:
    if data.get(key) is None:
        raise InvalidCellError(f'{where}: falta "{key}"')
    return data[key]


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidCellError(f"{where} debería ser un número, no {value!r}")
    return float(value)


def _numbers(value: Any, where: str, length: Optional[int] = None) -> Tuple[float, ...]:
    if not isinstance(value, list):
        raise InvalidCellError(f"{where} debería ser una lista de números")
    if length is not None and len(value) != length:
        raise InvalidCellError(f"{where} debería tener {length} números, tiene {len(value)}")
    return tuple(_number(v, f"{where}[{i}]") for i, v in enumerate(value))


def _point(value: Any, where: str) -> Point:
    return Point(*_numbers(value, where, length=3))


def _boolean(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise InvalidCellError(f"{where} debería ser true o false, no {value!r}")
    return value


def _build(where: str, factory):
    """Las primitivas validan sus medidas con ValueError: se reetiqueta con
    el sitio del fichero."""
    try:
        return factory()
    except ValueError as error:
        raise InvalidCellError(f"{where}: {error}") from error
