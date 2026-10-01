"""Capacidades de una célula y operaciones disponibles según el estado.

Dos niveles, a propósito separados (ver "Roles del Commander" en el vault):

- **Capacidades**: lo que la célula PUEDE hacer por cómo está montada --
  con o sin pinza, en simulación o en real. Las declara cada adaptador
  (`adapters.py`) junto a su código, y no cambian mientras la célula
  exista.
- **Operaciones disponibles**: lo que se puede hacer AHORA, según el estado
  del mundo (sujetando algo o no), con las opciones válidas de cada
  parámetro (qué cuerpos se pueden coger, a qué puntos o posturas se puede
  ir). Es lo que el servidor MCP necesita para decidir qué tools ofrecer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Iterable, Optional, Tuple

# --- Vocabulario de capacidades --------------------------------------------------

ARM_JOINTS = "arm.joints"  # mover el brazo en espacio articular (posturas)
ARM_CARTESIAN = "arm.cartesian"  # mover la brida a una pose, también en recta
GRIPPER_ACTUATE = "gripper.actuate"  # abrir y cerrar
GRIPPER_GRASP_DETECTION = "gripper.grasp_detection"  # saber si sujeta algo
SIM_GROUND_TRUTH = "sim.ground_truth"  # leer la pose exacta de cualquier cuerpo
SIM_RESET = "sim.reset"  # volver a construir la escena desde la descripción
PICK_AND_PLACE = "manipulation.pick_place"  # coger y dejar cuerpos (derivada)

DESCRIPTIONS: Dict[str, str] = {
    ARM_JOINTS: "Mover el brazo en espacio articular, p. ej. a una postura con nombre.",
    ARM_CARTESIAN: "Mover la brida a una pose cartesiana, también en línea recta.",
    GRIPPER_ACTUATE: "Abrir y cerrar la pinza.",
    GRIPPER_GRASP_DETECTION: "Saber si la pinza sujeta algo tras cerrar.",
    SIM_GROUND_TRUTH: "Leer del simulador la pose exacta de cualquier cuerpo (solo simulación).",
    SIM_RESET: "Reconstruir la escena desde su descripción (solo simulación).",
    PICK_AND_PLACE: "Coger un cuerpo de la escena y dejarlo en un punto.",
}

# El robot simulado no es un adaptador del registro: siempre es
# CoppeliaSimRobotAdapter, y el simulador aporta lo suyo.
SIM_ROBOT = frozenset({ARM_JOINTS, ARM_CARTESIAN, SIM_GROUND_TRUTH, SIM_RESET})


def derive(capabilities: Iterable[str]) -> FrozenSet[str]:
    """Añade las capacidades que salen de combinar otras."""
    result = set(capabilities)
    if {ARM_CARTESIAN, GRIPPER_ACTUATE, GRIPPER_GRASP_DETECTION} <= result:
        result.add(PICK_AND_PLACE)
    return frozenset(result)


# --- Operaciones disponibles ahora -------------------------------------------------


def choice(values: Iterable[str], description: str) -> Dict[str, Any]:
    """Parámetro de opciones cerradas: el LLM elige entre lo que existe."""
    return {"type": "string", "enum": list(values), "description": description}


def number(description: str) -> Dict[str, Any]:
    return {"type": "number", "description": description}


def text(description: str) -> Dict[str, Any]:
    return {"type": "string", "description": description}


_X = number("x en metros, en el marco de la base del robot")
_Y = number("y en metros, en el marco de la base del robot")
_Z = number("z en metros, en el marco de la base del robot (la mesa de mesa_cubo está en z = 0)")


@dataclass(frozen=True)
class Operation:
    """Una operación que se puede pedir ahora. `parameters` da el esquema
    JSON de cada parámetro (opciones cerradas como `enum`, números, texto):
    es directamente el `inputSchema` de su tool. `requires` es la capacidad
    de la que sale (None: solo necesita la célula abierta)."""

    name: str
    requires: Optional[str]
    description: str
    parameters: Tuple[Tuple[str, Dict[str, Any]], ...] = ()


def available_operations(
    capabilities: FrozenSet[str],
    postures: Iterable[str],
    points: Iterable[str],
    graspable_bodies: Iterable[str],
    holding: Optional[bool],
    held_body: Optional[str] = None,
) -> Tuple[Operation, ...]:
    """Las operaciones que tienen sentido en este estado. `holding` es None
    si no se sabe (sin detección de agarre): entonces no se ofrece ni
    `pick` ni `place`, porque no habría forma de comprobar el resultado."""
    postures, points = tuple(postures), tuple(points)
    candidates = tuple(name for name in graspable_bodies if name != held_body)
    operations = [
        Operation("define_point", None,
                  "Añadir al mundo un punto con nombre (o moverlo si ya existe), para usarlo como destino.",
                  (("name", text("nombre del punto")), ("x", _X), ("y", _Y), ("z", _Z))),
    ]
    if ARM_JOINTS in capabilities and postures:
        operations.append(Operation("move_to_posture", ARM_JOINTS, "Llevar el brazo a una postura con nombre.",
                                    (("posture", choice(postures, "postura")),)))
    if ARM_CARTESIAN in capabilities:
        operations.append(Operation("move_to_position", ARM_CARTESIAN,
                                    "Llevar el punto de agarre (entre los dedos; sin pinza, la brida) a una "
                                    "posición, con la herramienta hacia abajo. El camino no es recto.",
                                    (("x", _X), ("y", _Y), ("z", _Z))))
        if points:
            operations.append(Operation("move_above_point", ARM_CARTESIAN,
                                        "Llevar la herramienta encima de un punto con nombre, mirando hacia abajo.",
                                        (("point", choice(points, "punto")),)))
    if GRIPPER_ACTUATE in capabilities:
        operations.append(Operation("open_gripper", GRIPPER_ACTUATE, "Abrir la pinza (suelta lo que sujete)."))
        if not holding:
            operations.append(Operation("close_gripper", GRIPPER_ACTUATE, "Cerrar la pinza."))
    if PICK_AND_PLACE in capabilities and holding is not None:
        if not holding and candidates:
            operations.append(Operation("pick", PICK_AND_PLACE, "Coger un cuerpo de la escena, desde arriba.",
                                        (("body", choice(candidates, "cuerpo a coger")),)))
        if holding and points:
            operations.append(Operation("place", PICK_AND_PLACE,
                                        "Dejar lo que se sujeta con su centro en un punto con nombre.",
                                        (("point", choice(points, "punto donde dejarlo")),)))
    if SIM_GROUND_TRUTH in capabilities:
        operations.append(Operation("refresh_world", SIM_GROUND_TRUTH,
                                    "Actualizar el mundo con las poses exactas del simulador."))
    if SIM_RESET in capabilities:
        operations.append(Operation("reset_cell", SIM_RESET, "Reconstruir la escena desde su descripción."))
    return tuple(operations)
