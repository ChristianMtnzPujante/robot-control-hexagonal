"""Formato de una ESCENA (`descriptions/scenes/*.yaml`): qué hay en el
mundo al empezar -- cuerpos sólidos, puntos con nombre, obstáculos y
planos. No sabe de robots ni de nodos: la misma escena puede usarse en
varias células. Produce una `Scene` del dominio.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

from shared_kernel import Body, Box, Cylinder, Plane, Point, Pose, Scene, Sphere, SphereObstacle

from .errors import InvalidCellError
from .schema import Field, Section, boolean, build, mapping, number, numbers, required, text

SCENE = Section(
    "Escena",
    "descriptions/scenes/<nombre>.yaml",
    (
        Field("bodies", "dict nombre → cuerpo", False, "Cuerpos sólidos (ver «Cuerpo»). Van a `Scene.bodies`."),
        Field("points", "dict nombre → [x, y, z] m", False, "Puntos con nombre, p. ej. destinos. Van a `Scene.objects`."),
        Field("obstacles", "dict nombre → obstáculo", False, "Esferas a evitar (ver «Obstáculo»). Van a `Scene.obstacles`."),
        Field("planes", "dict nombre → plano", False, "Planos (ver «Plano»). Van a `Scene.planes`."),
    ),
    "Todo en metros y en el marco de la base del robot (que en simulación es el del mundo).",
)

BODY = Section(
    "Cuerpo",
    "scene: bodies.<nombre>",
    (
        Field("shape", "`box` | `cylinder` | `sphere`", True, "Forma."),
        Field("size", "[x, y, z] m", "si `box`", "Medidas totales de la caja."),
        Field("radius", "m", "si `cylinder` o `sphere`", "Radio."),
        Field("height", "m", "si `cylinder`", "Altura del cilindro (eje en su z)."),
        Field("position", "[x, y, z] m", True, "Centro de la forma."),
        Field("rpy_degrees", "[roll, pitch, yaw] °", False, "Orientación, convención URDF (ejes fijos x, y, z).", "[0, 0, 0]"),
        Field("quaternion", "[qx, qy, qz, qw]", False, "Orientación como cuaternión (en vez de `rpy_degrees`)."),
        Field("graspable", "bool", False, "Si la pinza lo puede coger.", "false"),
        Field("color", "[r, g, b] 0..1", False, "Solo visual. Sin él: rojo si se puede coger, gris si es fijo."),
    ),
)

OBSTACLE = Section(
    "Obstáculo",
    "scene: obstacles.<nombre>",
    (
        Field("center", "[x, y, z] m", True, "Centro de la esfera."),
        Field("radius", "m", True, "Radio."),
    ),
)

PLANE = Section(
    "Plano",
    "scene: planes.<nombre>",
    (
        Field("point", "[x, y, z] m", True, "Un punto del plano."),
        Field("normal", "[x, y, z]", True, "Normal (vector, no posición)."),
    ),
)

_SHAPE_FIELDS = {"box": {"size"}, "cylinder": {"radius", "height"}, "sphere": {"radius"}}


def parse_scene(raw: Any, where: str) -> Scene:
    data = SCENE.check(raw or {}, where)
    scene = Scene.empty()
    for name, body in mapping(data.get("bodies") or {}, f"{where}.bodies").items():
        scene = scene.with_body(name, parse_body(body, f"{where}.bodies.{name}"))
    for name, point in mapping(data.get("points") or {}, f"{where}.points").items():
        scene = scene.with_object(name, point_from(point, f"{where}.points.{name}"))
    for name, obstacle in mapping(data.get("obstacles") or {}, f"{where}.obstacles").items():
        at = f"{where}.obstacles.{name}"
        obstacle = OBSTACLE.check(obstacle, at)
        scene = scene.with_obstacle(
            name,
            build(at, lambda: SphereObstacle(point_from(obstacle["center"], f"{at}.center"),
                                             number(obstacle["radius"], f"{at}.radius"))),
        )
    for name, plane in mapping(data.get("planes") or {}, f"{where}.planes").items():
        at = f"{where}.planes.{name}"
        plane = PLANE.check(plane, at)
        scene = scene.with_plane(
            name, Plane(point_from(plane["point"], f"{at}.point"), point_from(plane["normal"], f"{at}.normal"))
        )
    return scene


def parse_body(raw: Any, where: str) -> Body:
    data = BODY.check(raw, where)
    shape_name = text(data["shape"], f"{where}.shape")
    if shape_name not in _SHAPE_FIELDS:
        raise InvalidCellError(f'{where}.shape "{shape_name}" no es válida: {", ".join(_SHAPE_FIELDS)}')
    other_shapes = set().union(*_SHAPE_FIELDS.values()) - _SHAPE_FIELDS[shape_name]
    misplaced = sorted(other_shapes & set(data))
    if misplaced:
        raise InvalidCellError(f'{where}: {", ".join(misplaced)} no aplica a shape: {shape_name}')
    if "rpy_degrees" in data and "quaternion" in data:
        raise InvalidCellError(f"{where}: rpy_degrees o quaternion, no los dos")

    def shape():
        if shape_name == "box":
            return Box(*numbers(required(data, "size", where), f"{where}.size", length=3))
        if shape_name == "cylinder":
            return Cylinder(number(required(data, "radius", where), f"{where}.radius"),
                            number(required(data, "height", where), f"{where}.height"))
        return Sphere(number(required(data, "radius", where), f"{where}.radius"))

    x, y, z = numbers(data["position"], f"{where}.position", length=3)
    qx, qy, qz, qw = orientation_from(data, where)
    color = data.get("color")
    return Body(
        shape=build(where, shape),
        pose=Pose(x, y, z, qx, qy, qz, qw),
        graspable=boolean(data.get("graspable", False), f"{where}.graspable"),
        color=None if color is None else numbers(color, f"{where}.color", length=3),
    )


def point_from(value: Any, where: str) -> Point:
    return Point(*numbers(value, where, length=3))


def orientation_from(data: Dict[str, Any], where: str) -> Tuple[float, float, float, float]:
    if "quaternion" in data:
        return numbers(data["quaternion"], f"{where}.quaternion", length=4)
    rpy = numbers(data.get("rpy_degrees", [0, 0, 0]), f"{where}.rpy_degrees", length=3)
    return rpy_degrees_to_quaternion(*rpy)


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
