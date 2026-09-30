"""Adaptador de salida (driven adapter): una pinza importada desde URDF en
CoppeliaSim, movida en modo CINEMÁTICO -- mismo criterio que
CoppeliaSimRobotAdapter con el brazo: `setJointPosition`, sin física.

Implementa GripperPort por duck typing (typing.Protocol).

Una pinza de URDF como la Robotiq 2F-85 (ver assets/robotiq_2f_85/) tiene
UN joint que se manda (`driven_joint`) y varios `<mimic>` que lo copian con
un multiplicador. simURDF importa los joints pero, en modo cinemático, nada
propaga el mimic: por eso este adaptador escribe TODOS los joints, cada uno
con su multiplicador. Los datos concretos (qué joint, su ángulo de cierre y
los multiplicadores) salen del propio URDF -- ver
`gripper_joints_from_urdf` -- en vez de copiarse a mano aquí.

Diferencias con la Robotiq real (Robotiq2FGripperAdapter), a propósito:
- `set_opening` anima el cierre en `steps` pasos y vuelve al TERMINAR, no
  al aceptar la orden: sin física no hay nada que siga moviendo los dedos
  después. Quien sondee get_state() después (lift_and_grip_demo) verá la
  posición ya estable, que es lo mismo que acabaría viendo con la real.
- `holding_object` siempre es False: sin física no hay contacto que
  detectar. Agarrar de verdad un objeto en simulación es otro paso.
- `activate()` no hace nada: no hay calibración que simular.
"""

from __future__ import annotations

import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional

from shared_kernel import GripperState


@dataclass(frozen=True)
class GripperJoints:
    """Qué joints forman una pinza de URDF: el que se manda, su ángulo de
    cierre total, y cuánto se mueve cada uno (el propio driven_joint
    incluido, con multiplicador 1)."""

    driven_joint: str
    closed_angle: float
    multipliers: Dict[str, float]


def gripper_joints_from_urdf(urdf_path: str, driven_joint: str) -> GripperJoints:
    """Lee de `urdf_path` los `<mimic>` que siguen a `driven_joint` y el
    límite de éste. El ángulo de cierre es el extremo del límite más lejos
    de cero (0 = abierta en la 2F-85: `lower=0.0 upper=0.8`)."""
    root = ET.parse(urdf_path).getroot()
    multipliers: Dict[str, float] = {}
    closed_angle: Optional[float] = None
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name == driven_joint:
            limit = joint.find("limit")
            if limit is None:
                raise ValueError(f'"{driven_joint}" no tiene <limit> en {urdf_path}')
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            closed_angle = upper if abs(upper) >= abs(lower) else lower
            multipliers[name] = 1.0
            continue
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") == driven_joint:
            if float(mimic.get("offset", "0")) != 0.0:
                raise ValueError(f'mimic con offset no soportado: "{name}"')
            multipliers[name] = float(mimic.get("multiplier", "1"))
    if closed_angle is None:
        raise ValueError(f'no hay ningún joint "{driven_joint}" en {urdf_path}')
    return GripperJoints(driven_joint, closed_angle, multipliers)


class CoppeliaSimGripperAdapter:
    def __init__(
        self,
        sim,
        joints: GripperJoints,
        steps: int = 15,
        step_pause_seconds: float = 0.03,
    ):
        """`sim` es el objeto de `RemoteAPIClient(...).require("sim")` -- se
        recibe hecho (en vez de crear otro cliente) para compartir la
        conexión con quien construyó la escena."""
        self._sim = sim
        self._joints = joints
        self._steps = max(1, steps)
        self._step_pause_seconds = step_pause_seconds
        self._handles: Dict[str, int] = {
            name: sim.getObject(f"/{name}") for name in joints.multipliers
        }

    def activate(self) -> None:
        pass

    def set_opening(self, fraction: float) -> None:
        if not 0.0 <= fraction <= 1.0:
            raise ValueError(
                f"apertura fuera de rango [0,1]: {fraction} (0 = abierta, 1 = cerrada)"
            )
        start = self._current_fraction()
        for step in range(1, self._steps + 1):
            self._write(start + (fraction - start) * step / self._steps)
            if step < self._steps:
                time.sleep(self._step_pause_seconds)

    def get_state(self) -> GripperState:
        return GripperState(
            opening=self._current_fraction(),
            activated=True,
            holding_object=False,
            fault_code=0,
        )

    def close(self) -> None:
        # No-op por el mismo motivo que CoppeliaSimRobotAdapter.close: la
        # conexión y la escena son de quien las creó.
        pass

    def _current_fraction(self) -> float:
        angle = self._sim.getJointPosition(self._handles[self._joints.driven_joint])
        return min(1.0, max(0.0, angle / self._joints.closed_angle))

    def _write(self, fraction: float) -> None:
        angle = fraction * self._joints.closed_angle
        for name, multiplier in self._joints.multipliers.items():
            self._sim.setJointPosition(self._handles[name], multiplier * angle)
