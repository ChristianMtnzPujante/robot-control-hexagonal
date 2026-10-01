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
from typing import Dict, FrozenSet, Iterable, Optional, Tuple

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


@dataclass(frozen=True)
class Operation:
    """Una operación que se puede pedir ahora. `options` da, por parámetro,
    los valores válidos (para que el LLM elija entre lo que existe)."""

    name: str
    requires: str  # la capacidad de la que sale
    description: str
    options: Tuple[Tuple[str, Tuple[str, ...]], ...] = ()


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
    operations = []
    if ARM_JOINTS in capabilities and postures:
        operations.append(Operation("move_to_posture", ARM_JOINTS, "Llevar el brazo a una postura con nombre.",
                                    (("posture", postures),)))
    if ARM_CARTESIAN in capabilities and points:
        operations.append(Operation("move_above_point", ARM_CARTESIAN,
                                    "Llevar la herramienta encima de un punto con nombre.", (("point", points),)))
    if GRIPPER_ACTUATE in capabilities:
        operations.append(Operation("open_gripper", GRIPPER_ACTUATE, "Abrir la pinza (suelta lo que sujete)."))
        if not holding:
            operations.append(Operation("close_gripper", GRIPPER_ACTUATE, "Cerrar la pinza."))
    if PICK_AND_PLACE in capabilities and holding is not None:
        if not holding and candidates:
            operations.append(Operation("pick", PICK_AND_PLACE, "Coger un cuerpo de la escena.",
                                        (("body", candidates),)))
        if holding and points:
            operations.append(Operation("place", PICK_AND_PLACE, "Dejar lo que se sujeta en un punto con nombre.",
                                        (("point", points),)))
    if SIM_GROUND_TRUTH in capabilities:
        operations.append(Operation("refresh_world", SIM_GROUND_TRUTH,
                                    "Actualizar el mundo con las poses exactas del simulador."))
    if SIM_RESET in capabilities:
        operations.append(Operation("reset_cell", SIM_RESET, "Reconstruir la escena desde su descripción."))
    return tuple(operations)
