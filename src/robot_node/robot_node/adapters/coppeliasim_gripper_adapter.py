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
- El agarre es CINEMÁTICO, no físico, y solo si se le da `grasp` (una
  `GraspGeometry`) y los cuerpos que se pueden coger: al cerrar, si hay un
  cuerpo `graspable` entre las yemas, los dedos se paran al tocarlo, el
  cuerpo pasa a ser hijo de la pinza y `holding_object` es True. Al abrir,
  vuelve a colgar de donde estaba, en el sitio en que se suelte. Sin
  `grasp`, `holding_object` siempre es False.
- `activate()` no hace nada: no hay calibración que simular.
"""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from shared_kernel import Body, Box, Cylinder, GripperState, Sphere

Vector = Tuple[float, float, float]


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


@dataclass(frozen=True)
class GraspGeometry:
    """Dónde están las yemas de una pinza paralela, para el agarre
    cinemático. Es de la PINZA, no de la escena: para la 2F-85 se calculó
    de sus mallas de colisión (ver ROBOTIQ_2F_85_GRASP en
    `commander/coppeliasim_scene_builder.py`).

    El marco de agarre se construye con las posiciones de cuatro joints,
    que simURDF deja donde dice el URDF (el frame del shape raíz, en
    cambio, puede venir reorientado): origen en el punto medio de los dos
    nudillos, x de nudillo derecho a izquierdo, z hacia el punto medio de
    las puntas (hacia fuera de la pinza).

    - `pad_z_range`: de dónde a dónde llegan las yemas en z de ese marco.
    - `pad_half_width`: media anchura de las yemas en y.
    - `half_gap_by_fraction`: distancia del plano medio a la cara interior
      de cada yema, muestreada en aperturas equiespaciadas de 0 (abierta)
      a 1 (cerrada)."""

    left_knuckle_joint: str
    right_knuckle_joint: str
    left_tip_joint: str
    right_tip_joint: str
    pad_z_range: Tuple[float, float]
    pad_half_width: float
    half_gap_by_fraction: Tuple[float, ...]

    def fraction_at_half_gap(self, half_gap: float) -> float:
        """Apertura (0..1) a la que las caras interiores quedan a
        `half_gap` del plano medio, interpolando la tabla. Fuera de rango
        se recorta: más ancho que la pinza abierta da 0, nada da 1."""
        table = self.half_gap_by_fraction
        if half_gap >= table[0]:
            return 0.0
        last = len(table) - 1
        for i in range(last):
            high, low = table[i], table[i + 1]
            if low <= half_gap <= high:
                return (i + (high - half_gap) / (high - low)) / last
        return 1.0


def half_extent_along(shape, direction: Vector) -> float:
    """Media extensión de `shape` a lo largo de `direction` (unitario,
    expresado en el marco del propio cuerpo)."""
    ux, uy, uz = (abs(c) for c in direction)
    if isinstance(shape, Box):
        return 0.5 * (ux * shape.size_x + uy * shape.size_y + uz * shape.size_z)
    if isinstance(shape, Cylinder):
        return shape.radius * math.sqrt(max(0.0, 1.0 - uz * uz)) + 0.5 * shape.height * uz
    if isinstance(shape, Sphere):
        return shape.radius
    raise TypeError(f"forma no soportada: {type(shape).__name__}")


def _sub(a: Sequence[float], b: Sequence[float]) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Sequence[float], b: Sequence[float]) -> Vector:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a: Sequence[float]) -> Vector:
    norm = math.sqrt(_dot(a, a))
    return (a[0] / norm, a[1] / norm, a[2] / norm)


class CoppeliaSimGripperAdapter:
    def __init__(
        self,
        sim,
        joints: GripperJoints,
        steps: int = 15,
        step_pause_seconds: float = 0.03,
        grasp: Optional[GraspGeometry] = None,
        graspable_bodies: Optional[Dict[str, Body]] = None,
    ):
        """`sim` es el objeto de `RemoteAPIClient(...).require("sim")` -- se
        recibe hecho (en vez de crear otro cliente) para compartir la
        conexión con quien construyó la escena.

        `graspable_bodies` son los cuerpos (por nombre = alias en
        CoppeliaSim) que el agarre puede coger; solo cuenta con `grasp`."""
        self._sim = sim
        self._joints = joints
        self._steps = max(1, steps)
        self._step_pause_seconds = step_pause_seconds
        self._handles: Dict[str, int] = {
            name: sim.getObject(f"/{name}") for name in joints.multipliers
        }
        self._grasp = grasp
        self._graspable_bodies = dict(graspable_bodies or {}) if grasp else {}
        self._frame_handles: List[int] = (
            [
                sim.getObject(f"/{name}")
                for name in (
                    grasp.left_knuckle_joint,
                    grasp.right_knuckle_joint,
                    grasp.left_tip_joint,
                    grasp.right_tip_joint,
                )
            ]
            if grasp
            else []
        )
        # Cuerpo agarrado: (nombre, handle, padre anterior), o None.
        self._held: Optional[Tuple[str, int, int]] = None

    def activate(self) -> None:
        pass

    def set_opening(self, fraction: float) -> None:
        if not 0.0 <= fraction <= 1.0:
            raise ValueError(
                f"apertura fuera de rango [0,1]: {fraction} (0 = abierta, 1 = cerrada)"
            )
        start = self._current_fraction()
        if self._held is not None:
            if fraction >= start:
                return  # los dedos ya están contra el cuerpo: no cierran más
            self._release()
            self._animate(start, fraction)
            return
        if fraction > start:
            contact = self._find_contact()
            if contact is not None and fraction >= contact[1]:
                self._animate(start, contact[1])
                self._attach(contact[0])
                return
        self._animate(start, fraction)

    @property
    def held_body(self) -> Optional[str]:
        """Nombre del cuerpo agarrado, o None. Fuera de GripperPort: la real
        solo sabe QUE agarró algo, no qué."""
        return self._held[0] if self._held else None

    def get_state(self) -> GripperState:
        return GripperState(
            opening=self._current_fraction(),
            activated=True,
            holding_object=self._held is not None,
            fault_code=0,
        )

    def close(self) -> None:
        # No-op por el mismo motivo que CoppeliaSimRobotAdapter.close: la
        # conexión y la escena son de quien las creó.
        pass

    def _animate(self, start: float, end: float) -> None:
        for step in range(1, self._steps + 1):
            self._write(start + (end - start) * step / self._steps)
            if step < self._steps:
                time.sleep(self._step_pause_seconds)

    def _grasp_frame(self) -> Tuple[Vector, Vector, Vector, Vector]:
        """(origen, x, y, z) del marco de agarre en coordenadas de mundo."""
        left_k, right_k, left_t, right_t = (
            self._sim.getObjectPosition(h, -1) for h in self._frame_handles
        )
        origin = tuple(0.5 * (a + b) for a, b in zip(left_k, right_k))
        x = _unit(_sub(left_k, right_k))
        tips = tuple(0.5 * (a + b) for a, b in zip(left_t, right_t))
        towards_tips = _sub(tips, origin)
        z = _unit(_sub(towards_tips, tuple(_dot(towards_tips, x) * c for c in x)))
        return origin, x, _cross(z, x), z

    def _find_contact(self) -> Optional[Tuple[str, float]]:
        """El primer cuerpo que se puede coger y está entre las yemas, con
        la apertura a la que la yema más cercana lo toca."""
        if not self._graspable_bodies:
            return None
        grasp = self._grasp
        origin, x, y, z = self._grasp_frame()
        best: Optional[Tuple[str, float]] = None
        for name, body in self._graspable_bodies.items():
            m = self._sim.getObjectMatrix(self._sim.getObject(f"/{name}"), -1)
            offset = _sub((m[3], m[7], m[11]), origin)
            px, py, pz = _dot(offset, x), _dot(offset, y), _dot(offset, z)
            # x del marco de agarre expresada en el marco del cuerpo: filas
            # de su matriz de rotación por x.
            x_in_body = (
                m[0] * x[0] + m[4] * x[1] + m[8] * x[2],
                m[1] * x[0] + m[5] * x[1] + m[9] * x[2],
                m[2] * x[0] + m[6] * x[1] + m[10] * x[2],
            )
            reach = abs(px) + half_extent_along(body.shape, x_in_body)
            inside = (
                grasp.pad_z_range[0] <= pz <= grasp.pad_z_range[1]
                and abs(py) <= grasp.pad_half_width
                and reach < grasp.half_gap_by_fraction[0]
            )
            if inside:
                fraction = grasp.fraction_at_half_gap(reach)
                if best is None or fraction < best[1]:
                    best = (name, fraction)
        return best

    def _attach(self, name: str) -> None:
        handle = self._sim.getObject(f"/{name}")
        previous_parent = self._sim.getObjectParent(handle)
        # Hijo del joint de la punta izquierda: se mueve con la pinza y el
        # brazo sin que nadie tenga que moverlo a mano.
        self._sim.setObjectParent(handle, self._frame_handles[2], True)
        self._held = (name, handle, previous_parent)

    def _release(self) -> None:
        _, handle, previous_parent = self._held
        self._sim.setObjectParent(handle, previous_parent, True)
        self._held = None

    def _current_fraction(self) -> float:
        angle = self._sim.getJointPosition(self._handles[self._joints.driven_joint])
        return min(1.0, max(0.0, angle / self._joints.closed_angle))

    def _write(self, fraction: float) -> None:
        angle = fraction * self._joints.closed_angle
        for name, multiplier in self._joints.multipliers.items():
            self._sim.setJointPosition(self._handles[name], multiplier * angle)
