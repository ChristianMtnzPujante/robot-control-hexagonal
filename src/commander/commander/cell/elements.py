"""Formatos de los elementos físicos reutilizables: un ROBOT
(`descriptions/robots/*.yaml`) y una HERRAMIENTA
(`descriptions/tools/*.yaml`). Sustituyen al antiguo catálogo en Python:
añadir un robot o una pinza es escribir un YAML, no tocar código -- salvo
que necesite un adaptador nuevo, que se nombra aquí (`adapter: ...`) y
vive en `adapters.py`.

Solo datos: dónde está su URDF, cómo se monta, la geometría de agarre...
Ni CoppeliaSim ni el robot hacen falta para leerlos.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from .errors import InvalidCellError
from .scene_format import rpy_degrees_to_quaternion
from .schema import Field, Section, mapping, number, numbers, path_relative_to, text, texts

# descriptions/ vive en la raíz del repo (resolve(): con symlink-install,
# __file__ apunta a build/ y parents no sería la raíz).
DESCRIPTIONS_DIR = Path(__file__).resolve().parents[4] / "descriptions"


# --- Robot -----------------------------------------------------------------------

ROBOT = Section(
    "Robot",
    "descriptions/robots/<nombre>.yaml",
    (
        Field("urdf", "ruta", True, "URDF del robot. Relativa al propio YAML, o con `~`."),
        Field("package_prefix", "ruta", True, "Directorio que sustituye a `package://` en las mallas del URDF."),
        Field("base_link", "texto", True, "Primer eslabón de la cadena serie (cinemática)."),
        Field("tip_link", "texto", True, "Último eslabón de la cadena serie: la brida."),
        Field("joint_names", "lista de textos", False, "Joints en orden. Si falta, se derivan del URDF (base_link → tip_link)."),
        Field("postures", "dict nombre → grados", False, "Posturas propias del robot (p. ej. `home`), en orden de joints."),
        Field("sim", "sección", False, "Datos para CoppeliaSim (ver «Robot: sim»)."),
        Field("real", "sección", False, "Datos para el robot real (ver «Robot: real»). Sin ella, solo simulación."),
    ),
)

ROBOT_SIM = Section(
    "Robot: sim",
    "robot: sim",
    (
        Field("root_alias", "texto", True, "Objeto raíz que crea simURDF al importar (para borrarlo al reconstruir)."),
        Field("tip_object", "texto", False, "Objeto de CoppeliaSim que hace de punta (rastro de waypoints)."),
    ),
)

ROBOT_REAL = Section(
    "Robot: real",
    "robot: real",
    (
        Field("adapter", "nombre", True, "Adaptador de conexión (`adapters.py`), p. ej. `cr5_tcp`."),
        Field("joint_limits_degrees", "lista de grados", False, "Límite por joint. Solo puede ESTRECHAR el de fábrica."),
    ),
)


@dataclass(frozen=True)
class RobotModel:
    name: str
    urdf_path: str
    package_prefix: str
    base_link: str
    tip_link: str
    joint_names: Tuple[str, ...]
    postures: Dict[str, Tuple[float, ...]] = field(default_factory=dict)
    sim_root_alias: Optional[str] = None
    sim_tip_object: Optional[str] = None
    real_adapter: Optional[str] = None
    real_joint_limits_degrees: Optional[Tuple[float, ...]] = None


def parse_robot(raw: Any, name: str, base_dir: Path) -> RobotModel:
    where = f"robots/{name}"
    data = ROBOT.check(raw, where)
    urdf = path_relative_to(data["urdf"], base_dir, f"{where}.urdf")
    base_link = text(data["base_link"], f"{where}.base_link")
    tip_link = text(data["tip_link"], f"{where}.tip_link")
    if data.get("joint_names") is not None:
        joint_names = texts(data["joint_names"], f"{where}.joint_names")
    else:
        joint_names = _joints_from_urdf(urdf, base_link, tip_link, where)
    postures = _postures(data.get("postures"), f"{where}.postures", len(joint_names))
    sim = ROBOT_SIM.check(data["sim"], f"{where}.sim") if data.get("sim") is not None else {}
    real = ROBOT_REAL.check(data["real"], f"{where}.real") if data.get("real") is not None else {}
    limits = real.get("joint_limits_degrees")
    if limits is not None:
        limits = numbers(limits, f"{where}.real.joint_limits_degrees", length=len(joint_names))
    return RobotModel(
        name=name,
        urdf_path=urdf,
        package_prefix=path_relative_to(data["package_prefix"], base_dir, f"{where}.package_prefix").rstrip("/") + "/",
        base_link=base_link,
        tip_link=tip_link,
        joint_names=joint_names,
        postures=postures,
        sim_root_alias=sim.get("root_alias"),
        sim_tip_object=sim.get("tip_object"),
        real_adapter=real.get("adapter"),
        real_joint_limits_degrees=limits,
    )


def _joints_from_urdf(urdf: str, base_link: str, tip_link: str, where: str) -> Tuple[str, ...]:
    from urdf_kit import parse_urdf_file

    try:
        description = parse_urdf_file(urdf, base_link, tip_link)
    except Exception as error:  # urdf_kit lanza tipos distintos según el fallo
        raise InvalidCellError(f"{where}: no pude sacar los joints del URDF ({error})") from error
    return tuple(joint.name for joint in description.joints)


def _postures(raw: Any, where: str, joint_count: int) -> Dict[str, Tuple[float, ...]]:
    return {
        name: numbers(values, f"{where}.{name}", length=joint_count)
        for name, values in mapping(raw or {}, where).items()
    }


# --- Herramienta -----------------------------------------------------------------

TOOL = Section(
    "Herramienta",
    "descriptions/tools/<nombre>.yaml",
    (
        Field("urdf", "ruta", True, "URDF de la herramienta, con su raíz libre para colgarla de una brida."),
        Field("package_prefix", "ruta", True, "Directorio que sustituye a `package://` en sus mallas."),
        Field("driven_joint", "texto", True, "El joint que se manda. Los `<mimic>` que lo siguen se leen del URDF."),
        Field("mounts", "dict robot → montaje", True, "Dónde se monta en cada robot (ver «Herramienta: montaje»)."),
        Field("grasp", "sección", True, "Cómo agarra (ver «Herramienta: agarre»)."),
        Field("sim", "sección", False, "Adaptador en simulación (ver «Herramienta: sim»)."),
        Field("real", "sección", False, "Adaptador en el robot real (ver «Herramienta: real»)."),
    ),
)

TOOL_MOUNT = Section(
    "Herramienta: montaje",
    "tool: mounts.<robot>",
    (
        Field("parent_joint", "texto", True, "Joint del robot del que cuelga (la brida)."),
        Field("position", "[x, y, z] m", False, "Desplazamiento respecto a ese joint con el robot a cero (acoplador).", "[0, 0, 0]"),
        Field("rpy_degrees", "[roll, pitch, yaw] °", False, "Giro respecto a ese joint.", "[0, 0, 0]"),
    ),
)

TOOL_GRASP = Section(
    "Herramienta: agarre",
    "tool: grasp",
    (
        Field("offset", "m", True, "Distancia de la brida al centro de lo agarrado, según el MODELO. En real, la célula debe dar la medida."),
        Field("pads", "sección", False, "Geometría de las yemas para el agarre cinemático en simulación (ver «Herramienta: yemas»)."),
    ),
)

TOOL_PADS = Section(
    "Herramienta: yemas",
    "tool: grasp.pads",
    (
        Field("frame_joints", "dict", True, "`left_knuckle`, `right_knuckle`, `left_tip`, `right_tip`: joints con los que se construye el marco de agarre."),
        Field("z_range", "[mín, máx] m", True, "Altura de las yemas en ese marco (origen entre los nudillos, z hacia las puntas)."),
        Field("half_width", "m", True, "Media anchura de las yemas."),
        Field("half_gap_by_fraction", "lista de m", True, "Distancia del plano medio a la cara interior, de abierta (0) a cerrada (1), equiespaciada."),
    ),
)

TOOL_SIM = Section("Herramienta: sim", "tool: sim", (
    Field("adapter", "nombre", True, "Adaptador en CoppeliaSim (`adapters.py`), p. ej. `coppeliasim_urdf_gripper`."),
))

TOOL_REAL = Section("Herramienta: real", "tool: real", (
    Field("adapter", "nombre", True, "Adaptador real (`adapters.py`), p. ej. `robotiq_modbus_flange`."),
))

_FRAME_JOINT_KEYS = ("left_knuckle", "right_knuckle", "left_tip", "right_tip")


@dataclass(frozen=True)
class ToolMountSpec:
    parent_joint: str
    offset_pose: Tuple[float, ...]  # [x, y, z, qx, qy, qz, qw]


@dataclass(frozen=True)
class GraspPads:
    frame_joints: Tuple[str, str, str, str]  # nudillo izq., nudillo der., punta izq., punta der.
    z_range: Tuple[float, float]
    half_width: float
    half_gap_by_fraction: Tuple[float, ...]


@dataclass(frozen=True)
class ToolModel:
    name: str
    urdf_path: str
    package_prefix: str
    driven_joint: str
    mounts: Dict[str, ToolMountSpec]
    grasp_offset: float
    pads: Optional[GraspPads] = None
    sim_adapter: Optional[str] = None
    real_adapter: Optional[str] = None


def parse_tool(raw: Any, name: str, base_dir: Path) -> ToolModel:
    where = f"tools/{name}"
    data = TOOL.check(raw, where)
    mounts = {}
    for robot, mount in mapping(data["mounts"], f"{where}.mounts").items():
        at = f"{where}.mounts.{robot}"
        mount = TOOL_MOUNT.check(mount, at)
        position = numbers(mount.get("position", [0, 0, 0]), f"{at}.position", length=3)
        rpy = numbers(mount.get("rpy_degrees", [0, 0, 0]), f"{at}.rpy_degrees", length=3)
        mounts[robot] = ToolMountSpec(
            text(mount["parent_joint"], f"{at}.parent_joint"), position + rpy_degrees_to_quaternion(*rpy)
        )
    grasp = TOOL_GRASP.check(data["grasp"], f"{where}.grasp")
    pads = None
    if grasp.get("pads") is not None:
        at = f"{where}.grasp.pads"
        raw_pads = TOOL_PADS.check(grasp["pads"], at)
        frame = mapping(raw_pads["frame_joints"], f"{at}.frame_joints")
        missing = [key for key in _FRAME_JOINT_KEYS if key not in frame]
        unknown = sorted(set(frame) - set(_FRAME_JOINT_KEYS))
        if missing or unknown:
            raise InvalidCellError(f"{at}.frame_joints necesita exactamente {', '.join(_FRAME_JOINT_KEYS)}")
        gaps = numbers(raw_pads["half_gap_by_fraction"], f"{at}.half_gap_by_fraction")
        if len(gaps) < 2 or any(b > a for a, b in zip(gaps, gaps[1:])):
            raise InvalidCellError(f"{at}.half_gap_by_fraction: al menos 2 valores, de abierta a cerrada (decrecientes)")
        pads = GraspPads(
            frame_joints=tuple(text(frame[key], f"{at}.frame_joints.{key}") for key in _FRAME_JOINT_KEYS),
            z_range=numbers(raw_pads["z_range"], f"{at}.z_range", length=2),
            half_width=number(raw_pads["half_width"], f"{at}.half_width"),
            half_gap_by_fraction=gaps,
        )
    sim = TOOL_SIM.check(data["sim"], f"{where}.sim") if data.get("sim") is not None else {}
    real = TOOL_REAL.check(data["real"], f"{where}.real") if data.get("real") is not None else {}
    return ToolModel(
        name=name,
        urdf_path=path_relative_to(data["urdf"], base_dir, f"{where}.urdf"),
        package_prefix=path_relative_to(data["package_prefix"], base_dir, f"{where}.package_prefix").rstrip("/") + "/",
        driven_joint=text(data["driven_joint"], f"{where}.driven_joint"),
        mounts=mounts,
        grasp_offset=number(grasp["offset"], f"{where}.grasp.offset"),
        pads=pads,
        sim_adapter=sim.get("adapter"),
        real_adapter=real.get("adapter"),
    )


# --- Lectura por nombre ----------------------------------------------------------


def resolve_ref(kind: str, ref: str, base_dir: Optional[Path] = None) -> Path:
    """`ref` es un nombre de `descriptions/<kind>/` (`cr5`) o una ruta a un
    .yaml, relativa al fichero que la cita."""
    if ref.endswith((".yaml", ".yml")) or "/" in ref:
        path = Path(ref).expanduser()
        if not path.is_absolute() and base_dir is not None:
            path = base_dir / path
    else:
        path = DESCRIPTIONS_DIR / kind / f"{ref}.yaml"
    if not path.is_file():
        available = ", ".join(sorted(p.stem for p in (DESCRIPTIONS_DIR / kind).glob("*.yaml")))
        raise InvalidCellError(f'no encuentro "{ref}" en descriptions/{kind}/. Hay: {available or "nada"}')
    return path.resolve()


def read_yaml(path: Path) -> Any:
    with open(path) as file:
        return yaml.safe_load(file)


def load_robot(ref: str, base_dir: Optional[Path] = None) -> RobotModel:
    path = resolve_ref("robots", ref, base_dir)
    return parse_robot(read_yaml(path), path.stem, path.parent)


def load_tool(ref: str, base_dir: Optional[Path] = None) -> ToolModel:
    path = resolve_ref("tools", ref, base_dir)
    return parse_tool(read_yaml(path), path.stem, path.parent)
