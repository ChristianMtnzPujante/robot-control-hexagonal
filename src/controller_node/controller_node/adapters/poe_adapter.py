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

IK por Newton-Raphson sobre el Jacobiano espacial vía la Adjunta, tal como
se dedujo a mano en las sesiones de teoría (Lynch & Park, *Modern
Robotics*, cap. 6, `IKinSpace`): en cada iteración se calcula el twist de
error en frame espacial Vs = Ad_{T_sb}·log(T_sb⁻¹·T_sd) y se actualiza
θ. El paso usa mínimos cuadrados amortiguados (Levenberg-Marquardt /
damped least squares, Wampler 1986) en vez de la pseudoinversa pura de
J_s(θ): θ ← θ + J_sᵀ(J_s J_sᵀ + λ²I)⁻¹Vs, con λ² escalado por ‖Vs‖² para que
la amortiguación desaparezca según se acerca a la solución. Sigue siendo
"Newton-Raphson sobre el Jacobiano" -- es la estabilización numérica
estándar del mismo paso, necesaria porque el CR5 tiene muñeca esférica
(joints 4-5-6 con ejes que se cortan en un punto) y por tanto una
singularidad de muñeca real en joint5≈0 (los ejes de joint4 y joint6 quedan
paralelos ahí, ver también docs/algebra_geometrica_conforme.md §5 sobre la
misma muñeca esférica); sin amortiguación, la pseudoinversa puede disparar
el paso al pasar cerca. Si aun así no converge en el número de iteraciones
dado, se lanza un error explícito, igual que `coppeliasim_ik_adapter.py`
cuando `simIK` no converge.

Ver docs/algebra_geometrica_conforme.md §4: estos mismos twists (t_i, B_i
en notación CGA) son la entrada que necesitaría `GaKinematicsAdapter` --
extraerlos aquí ya cubre ese trabajo para cuando se implemente ese
adaptador, no hace falta re-derivarlos aparte.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory

# Orden de las articulaciones tal como lo declaran robot_node (ver
# robot_node/node.py) y el URDF: joint1..joint6, revolutas, eje Z local.
_JOINT_NAMES: Tuple[str, ...] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)

# (xyz, rpy) de cada <joint><origin> del URDF del CR5, en orden de cadena
# cinemática parent->child. El <axis> de las seis es siempre "0 0 1" local,
# así que no hace falta guardarlo aparte.
#
# TODO: hardcodeado a mano para el CR5 concreto (copiado del URDF en
# ~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/cr5_robot.urdf).
# Para soportar otro robot sin tocar este archivo, lo general sería parsear
# el .urdf en tiempo de carga (p.ej. con `urdf_parser_py`) y extraer
# xyz/rpy/axis de cada <joint> automáticamente, en vez de mantener esta
# constante a mano por robot.
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


def _se3_exp(screw: np.ndarray, theta: float) -> np.ndarray:
    """exp([S]θ) -- fórmula de Rodrigues extendida a se(3) (Modern Robotics
    ec. 3.51). `screw`=(w,v) con |w|=1: las seis articulaciones del CR5 son
    revolutas, nunca prismáticas."""
    w, v = screw[:3], screw[3:]
    w_hat = _skew(w)
    w_hat_sq = w_hat @ w_hat
    rotation = np.eye(3) + math.sin(theta) * w_hat + (1 - math.cos(theta)) * w_hat_sq
    g = (
        np.eye(3) * theta
        + (1 - math.cos(theta)) * w_hat
        + (theta - math.sin(theta)) * w_hat_sq
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = g @ v
    return transform


def _forward_kinematics(thetas: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    for screw, theta in zip(_HOME_SCREW_AXES, thetas):
        transform = transform @ _se3_exp(screw, theta)
    return transform @ _HOME_POSE


def _adjoint(transform: np.ndarray) -> np.ndarray:
    rotation, p = transform[:3, :3], transform[:3, 3]
    ad = np.zeros((6, 6))
    ad[:3, :3] = rotation
    ad[3:, :3] = _skew(p) @ rotation
    ad[3:, 3:] = rotation
    return ad


def _jacobian_space(thetas: np.ndarray) -> np.ndarray:
    """J_s(θ) columna a columna: J_i = Ad_{T_{i-1}}·S_i, con T_{i-1} el
    producto de exponenciales hasta (sin incluir) la articulación i (Modern
    Robotics ec. 5.11)."""
    jacobian = np.zeros((6, 6))
    transform = np.eye(4)
    for i, (screw, theta) in enumerate(zip(_HOME_SCREW_AXES, thetas)):
        jacobian[:, i] = _adjoint(transform) @ screw
        transform = transform @ _se3_exp(screw, theta)
    return jacobian


def _matrix_log_so3(rotation: np.ndarray) -> np.ndarray:
    """log(R) como matriz antisimétrica θ·[ŵ] (Modern Robotics ec. 3.58)."""
    cos_theta = min(1.0, max(-1.0, (np.trace(rotation) - 1) / 2))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.zeros((3, 3))
    if abs(theta - math.pi) < 1e-9:
        # R es simétrica en este caso -- cualquier columna de (R+I) sirve de eje.
        symmetric = rotation + np.eye(3)
        axis = symmetric[:, int(np.argmax(np.diagonal(symmetric)))]
        axis = axis / np.linalg.norm(axis)
        return _skew(axis) * theta
    return theta / (2 * math.sin(theta)) * (rotation - rotation.T)


def _matrix_log_se3(transform: np.ndarray) -> np.ndarray:
    """log(T) como vector de 6 -- [w;v]·θ (Modern Robotics ec. 3.88). Es lo
    que da el twist de error entre la pose actual y la pose objetivo en
    cada iteración de Newton-Raphson."""
    rotation, p = transform[:3, :3], transform[:3, 3]
    omega_hat_theta = _matrix_log_so3(rotation)
    if np.allclose(omega_hat_theta, 0.0):
        return np.concatenate([np.zeros(3), p])
    cos_theta = min(1.0, max(-1.0, (np.trace(rotation) - 1) / 2))
    theta = math.acos(cos_theta)
    omega_hat_theta_sq = omega_hat_theta @ omega_hat_theta
    g_inv = (
        np.eye(3)
        - omega_hat_theta / 2
        + (1 / theta - 0.5 / math.tan(theta / 2)) / theta * omega_hat_theta_sq
    )
    v_theta = g_inv @ p
    w_theta = np.array(
        [omega_hat_theta[2, 1], omega_hat_theta[0, 2], omega_hat_theta[1, 0]]
    )
    return np.concatenate([w_theta, v_theta])


def _quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _quaternion_to_matrix(pose.qx, pose.qy, pose.qz, pose.qw)
    transform[:3, 3] = [pose.x, pose.y, pose.z]
    return transform


class PoeKinematicsAdapter:
    def __init__(
        self,
        max_iterations: int = 200,
        orientation_tolerance: float = 1e-3,
        position_tolerance: float = 1e-4,
        damping_factor: float = 1e-2,
        steps: int = 20,
    ):
        self._max_iterations = max_iterations
        self._orientation_tolerance = orientation_tolerance
        self._position_tolerance = position_tolerance
        self._damping_factor = damping_factor
        self._steps = steps

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        target_configuration = self._inverse_kinematics(goal, current_configuration)
        return Trajectory.straight_line(
            current_configuration, target_configuration, self._steps
        )

    def _inverse_kinematics(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> JointConfiguration:
        goal_transform = _pose_to_matrix(goal)
        thetas = np.array(
            [current_configuration.angle_of(name) for name in _JOINT_NAMES]
        )

        for _ in range(self._max_iterations):
            current_transform = _forward_kinematics(thetas)
            error_body = _matrix_log_se3(
                np.linalg.inv(current_transform) @ goal_transform
            )
            error_space = _adjoint(current_transform) @ error_body
            if (
                np.linalg.norm(error_space[:3]) < self._orientation_tolerance
                and np.linalg.norm(error_space[3:]) < self._position_tolerance
            ):
                positions = [
                    JointPosition(name, float(theta))
                    for name, theta in zip(_JOINT_NAMES, thetas)
                ]
                return JointConfiguration.create(positions).value
            jacobian = _jacobian_space(thetas)
            damping_sq = self._damping_factor * float(error_space @ error_space)
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping_sq * np.eye(6), error_space
            )
            thetas = thetas + step

        raise RuntimeError(
            "PoeKinematicsAdapter: Newton-Raphson no convergió en "
            f"{self._max_iterations} iteraciones para "
            f"Pose(x={goal.x}, y={goal.y}, z={goal.z}). "
            "Prueba un objetivo más cercano a la configuración actual del brazo."
        )
