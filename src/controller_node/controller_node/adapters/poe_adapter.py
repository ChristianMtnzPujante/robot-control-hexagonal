"""Cinemática vía Product of Exponentials (Explicit-Robotics, Nah & Lachner).

Twists S_i=(w_i, v_i) extraídos de los `<origin>`/`<axis>` de cada `<joint>`
del URDF del CR5 (`~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/
cr5_robot.urdf`, el mismo mecanismo que la escena `cr5_base.ttt` ya
importada en CoppeliaSim), en su configuración home (todos los ángulos a
cero): las seis articulaciones son revolutas con eje local Z, así que cada
twist en el frame espacial (`base_link`) es w_i = R_i·ẑ, v_i = -w_i × q_i,
con (R_i, q_i) la orientación/posición home del frame de la articulación i,
acumulando las transformadas `origin` del URDF en orden de cadena.
`_JOINT_ORIGINS` son esos valores tal cual constan en el URDF -- así el
cálculo de los twists queda auditable contra la fuente en vez de
esconderse en constantes ya derivadas. `M` (`_HOME_POSE`) es la pose home
de `Link6` (mismo tip que usa `coppeliasim_ik_adapter.py` como
`Link6_visual`).

TODO — pendiente de verdad: con los twists S_i y la pose home M ya
extraídos (`_HOME_SCREW_AXES`, `_HOME_POSE`), falta implementar el bucle
Newton-Raphson con el Jacobiano vía la representación Adjunta, tal como se
dedujo a mano en las sesiones de teoría.

Ver docs/algebra_geometrica_conforme.md §4: estos mismos twists (t_i, B_i
en notación CGA) son la entrada que necesitaría `GaKinematicsAdapter` --
extraerlos aquí ya cubre ese trabajo para cuando se implemente ese
adaptador, no hace falta re-derivarlos aparte.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from shared_kernel import JointConfiguration, Pose, Trajectory

# (xyz, rpy) de cada <joint><origin> del URDF del CR5, en orden de cadena
# cinemática parent->child. El <axis> de las seis es siempre "0 0 1" local,
# así que no hace falta guardarlo aparte.
_JOINT_ORIGINS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.147), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.5708, 1.5708, 0.0)),
    ((-0.427, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-0.357, 0.0, 0.141), (0.0, 0.0, -1.5708)),
    ((0.0, -0.116, 0.0), (1.5708, 0.0, 0.0)),
    ((0.0, 0.105, 0.0), (-1.5708, 0.0, 0.0)),
)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) -- convención rpy de URDF (ejes fijos)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def _home_screw_axes() -> Tuple[List[np.ndarray], np.ndarray]:
    """Deriva los twists S_i y la pose home M acumulando `_JOINT_ORIGINS`
    (ver docstring del módulo)."""
    transform = np.eye(4)
    screws = []
    for xyz, rpy in _JOINT_ORIGINS:
        origin = np.eye(4)
        origin[:3, :3] = _rpy_to_matrix(*rpy)
        origin[:3, 3] = xyz
        transform = transform @ origin
        w = transform[:3, 2]
        q = transform[:3, 3]
        v = -np.cross(w, q)
        screws.append(np.concatenate([w, v]))
    return screws, transform


_HOME_SCREW_AXES, _HOME_POSE = _home_screw_axes()


class PoeKinematicsAdapter:
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        raise NotImplementedError(
            "PoeKinematicsAdapter: twists S_i y pose home M ya extraídos "
            "(_HOME_SCREW_AXES, _HOME_POSE) -- falta el bucle Newton-Raphson "
            "sobre el Jacobiano vía la Adjunta."
        )
