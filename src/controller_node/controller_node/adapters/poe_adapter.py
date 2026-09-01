"""Cinemática vía Product of Exponentials (Explicit-Robotics, Nah & Lachner).

Twists S_i=(w_i, v_i) derivados de un `RobotDescription` (shared_kernel) en
configuración home: w_i=R_i·axis_i, v_i=-w_i×q_i (o v_i=R_i·axis_i para
articulaciones prismáticas). IK por Newton-Raphson sobre el Jacobiano
espacial vía la Adjunta (Lynch & Park, *Modern Robotics*, cap. 6,
`IKinSpace`), con paso amortiguado (Levenberg-Marquardt, Wampler 1986) en
vez de pseudoinversa pura -- el CR5 tiene muñeca esférica y una
singularidad real en joint5≈0 (ejes de joint4 y joint6 paralelos ahí) donde
la pseudoinversa sin amortiguar dispara el paso. Derivación completa
función a función, con diagrama del mecanismo: ver `poe_adapter.html` en
este mismo directorio.

El `RobotDescription` por defecto (`_DEFAULT_CR5_DESCRIPTION`, más abajo)
reproduce los mismos datos que antes vivían hardcodeados aquí como
`_JOINT_NAMES`/`_JOINT_ORIGINS`, copiados del URDF real del CR5 (en
~/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/cr5_robot.urdf). Para
cargar un robot arbitrario en tiempo de carga, `controller_node/_build_adapter`
ya llama a `urdf_kit.parse_urdf_file` con los parámetros ROS2
`urdf_path`/`base_link`/`tip_link` y pasa el resultado como
`robot_description` (ver ROADMAP.md, Bloque 9). Mientras tanto,
`_validate_cr5_reference_if_applicable` (más abajo) es una red de seguridad
TEMPORAL que compara cualquier `RobotDescription` que diga ser un CR5 contra
`_DEFAULT_CR5_DESCRIPTION` -- borrar cuando esta ruta genérica lleve tiempo
validada.

Ver docs/algebra_geometrica_conforme.md §4: estos mismos twists son la
entrada que necesitaría `GaKinematicsAdapter` -- no hace falta
re-derivarlos aparte cuando se implemente; solo cambia el álgebra con la
que se interpretan los mismos datos crudos de `RobotDescription`.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from shared_kernel import (
    JointConfiguration,
    JointDescription,
    JointPosition,
    Pose,
    RobotDescription,
    Trajectory,
)

# Orden de las articulaciones tal como lo declaran robot_node (ver
# robot_node/node.py) y el URDF: joint1..joint6, revolutas, eje Z local.
_CR5_JOINT_NAMES: Tuple[str, ...] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)

# (xyz, rpy) de cada <joint><origin> del URDF del CR5, en orden de cadena
# cinemática parent->child (el CR5 no tiene <joint type="fixed"> intermedios
# entre articulaciones móviles, así que no hace falta plegado -- a
# diferencia de urdf_kit.parse_urdf_file, que sí lo hace para el caso
# general). El <axis> de las seis es siempre "0 0 1" local.
_CR5_JOINT_ORIGINS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.147), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.5708, 1.5708, 0.0)),
    ((-0.427, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-0.357, 0.0, 0.141), (0.0, 0.0, -1.5708)),
    ((0.0, -0.116, 0.0), (1.5708, 0.0, 0.0)),
    ((0.0, 0.105, 0.0), (-1.5708, 0.0, 0.0)),
)


def _default_cr5_description() -> RobotDescription:
    """Fallback histórico: mismos joint_names/orígenes que antes vivían en
    `_JOINT_NAMES`/`_JOINT_ORIGINS`, ahora tipados como `RobotDescription`.
    `base_link`/`tip_link` son nombres genéricos -- el cálculo de este
    adaptador no los usa, solo los exige `RobotDescription.create` como
    parte del value object; no tienen por qué coincidir con los nombres de
    escena de CoppeliaSim (`coppeliasim_ik_adapter.py` usa los suyos)."""
    joints = [
        JointDescription(
            name=name,
            joint_type="revolute",
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=(0.0, 0.0, 1.0),
        )
        for name, (xyz, rpy) in zip(_CR5_JOINT_NAMES, _CR5_JOINT_ORIGINS)
    ]
    # Datos internos fijos y conocidos válidos -- nunca Left.
    return RobotDescription.create(joints, base_link="base_link", tip_link="link6").value


_DEFAULT_CR5_DESCRIPTION = _default_cr5_description()


def _validate_cr5_reference_if_applicable(description: RobotDescription) -> None:
    """Red de seguridad TEMPORAL: si `description` dice ser un CR5 (mismos
    nombres de articulación que `_CR5_JOINT_NAMES`), comprueba que sus
    twists/pose home coincidan con `_DEFAULT_CR5_DESCRIPTION` -- la tabla
    hardcodeada que ya se validó a mano contra el URDF real. Objetivo:
    confiar en la ruta genérica de `urdf_kit.parse_urdf_file` para el único
    robot real que tenemos hoy, antes de fiarnos de ella para uno nuevo.
    Borrar junto con `_DEFAULT_CR5_DESCRIPTION` cuando esta ruta lleve
    tiempo validada (ver ROADMAP.md, Bloque 9)."""
    if description.joint_names != _CR5_JOINT_NAMES:
        return
    screws, home_pose = _screw_axes_from_description(description)
    reference_screws, reference_home_pose = _screw_axes_from_description(
        _DEFAULT_CR5_DESCRIPTION
    )
    for name, screw, reference_screw in zip(_CR5_JOINT_NAMES, screws, reference_screws):
        if not np.allclose(screw, reference_screw, atol=1e-6):
            raise ValueError(
                f'PoeKinematicsAdapter: el twist de "{name}" derivado del '
                "RobotDescription recibido no coincide con el de referencia "
                "del CR5 -- revisa urdf_path/base_link/tip_link."
            )
    if not np.allclose(home_pose, reference_home_pose, atol=1e-6):
        raise ValueError(
            "PoeKinematicsAdapter: la pose home derivada del RobotDescription "
            "recibido no coincide con la de referencia del CR5 -- revisa "
            "urdf_path/base_link/tip_link."
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


def _screw_axes_from_description(
    description: RobotDescription,
) -> Tuple[List[np.ndarray], np.ndarray]:
    """Deriva los twists S_i y la pose home M acumulando los orígenes de
    `description` (ver docstring del módulo). Generaliza la versión previa
    (que asumía eje Z local fijo, como el CR5): usa `joint.axis` real de
    cada articulación, y soporta `prismatic` además de `revolute`/`continuous`
    (Modern Robotics, tabla 3.3: w=0, v=axis para prismáticas)."""
    transform = np.eye(4)
    screws = []
    for joint in description.joints:
        origin = np.eye(4)
        origin[:3, :3] = _rpy_to_matrix(*joint.origin_rpy)
        origin[:3, 3] = joint.origin_xyz
        transform = transform @ origin
        axis_world = transform[:3, :3] @ np.array(joint.axis)
        if joint.joint_type == "prismatic":
            w = np.zeros(3)
            v = axis_world
        else:
            w = axis_world
            q = transform[:3, 3]
            v = -np.cross(w, q)
        screws.append(np.concatenate([w, v]))
    return screws, transform


def _se3_exp(screw: np.ndarray, theta: float) -> np.ndarray:
    """exp([S]θ) -- fórmula de Rodrigues extendida a se(3) (Modern Robotics
    ec. 3.51). `screw`=(w,v) con |w|=1 para articulaciones revolute/continuous;
    la misma fórmula vale sin cambios para prismatic (w=0: `rotation` queda
    la identidad y `g@v` se reduce a theta*v, la traslación pura)."""
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


def _forward_kinematics(
    screw_axes: List[np.ndarray], home_pose: np.ndarray, thetas: np.ndarray
) -> np.ndarray:
    transform = np.eye(4)
    for screw, theta in zip(screw_axes, thetas):
        transform = transform @ _se3_exp(screw, theta)
    return transform @ home_pose


def _adjoint(transform: np.ndarray) -> np.ndarray:
    rotation, p = transform[:3, :3], transform[:3, 3]
    ad = np.zeros((6, 6))
    ad[:3, :3] = rotation
    ad[3:, :3] = _skew(p) @ rotation
    ad[3:, 3:] = rotation
    return ad


def _jacobian_space(screw_axes: List[np.ndarray], thetas: np.ndarray) -> np.ndarray:
    """J_s(θ) columna a columna: J_i = Ad_{T_{i-1}}·S_i, con T_{i-1} el
    producto de exponenciales hasta (sin incluir) la articulación i (Modern
    Robotics ec. 5.11). Filas=6 siempre (dimensión de se(3), no depende del
    nº de articulaciones); columnas=len(screw_axes)."""
    jacobian = np.zeros((6, len(screw_axes)))
    transform = np.eye(4)
    for i, (screw, theta) in enumerate(zip(screw_axes, thetas)):
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


def _matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Inversa de `_quaternion_to_matrix` -- método de Shepperd (Modern
    Robotics, apéndice B): la rama con el mayor término en el denominador
    evita perder precisión cerca de ángulo=π. Necesaria para
    `PoeKinematicsAdapter.forward_kinematics`: expresar una pose calculada
    como `Pose` (cuaternión), no solo como matriz interna."""
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def _matrix_to_pose(transform: np.ndarray) -> Pose:
    qx, qy, qz, qw = _matrix_to_quaternion(transform[:3, :3])
    x, y, z = transform[:3, 3]
    return Pose(x=float(x), y=float(y), z=float(z), qx=qx, qy=qy, qz=qz, qw=qw)


class PoeKinematicsAdapter:
    def __init__(
        self,
        max_iterations: int = 200,
        orientation_tolerance: float = 1e-3,
        position_tolerance: float = 1e-4,
        damping_factor: float = 1e-2,
        steps: int = 20,
        robot_description: RobotDescription = _DEFAULT_CR5_DESCRIPTION,
    ):
        _validate_cr5_reference_if_applicable(robot_description)
        self._max_iterations = max_iterations
        self._orientation_tolerance = orientation_tolerance
        self._position_tolerance = position_tolerance
        self._damping_factor = damping_factor
        self._steps = steps
        self._joint_names = robot_description.joint_names
        self._screw_axes, self._home_pose = _screw_axes_from_description(
            robot_description
        )

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        target_configuration = self._inverse_kinematics(goal, current_configuration)
        return Trajectory.straight_line(
            current_configuration, target_configuration, self._steps
        )

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        """Cinemática directa: de una `JointConfiguration` a la `Pose`
        cartesiana del tip, relativa a `base_link` (mismo frame que exige
        `compute_trajectory` para `goal` -- ver two_sessions_demo.py). No es
        parte de `KinematicsPort` (ese puerto solo exige IK) -- lo usan
        adaptadores de `PlanningPort` que necesitan saber DÓNDE está el
        robot ahora en cartesiano antes de decidir por dónde desviarse
        (ver `obstacle_avoiding_planning_adapter.py`)."""
        thetas = np.array(
            [configuration.angle_of(name) for name in self._joint_names]
        )
        transform = _forward_kinematics(self._screw_axes, self._home_pose, thetas)
        return _matrix_to_pose(transform)

    def _inverse_kinematics(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> JointConfiguration:
        goal_transform = _pose_to_matrix(goal)
        thetas = np.array(
            [current_configuration.angle_of(name) for name in self._joint_names]
        )

        for _ in range(self._max_iterations):
            current_transform = _forward_kinematics(
                self._screw_axes, self._home_pose, thetas
            )
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
                    for name, theta in zip(self._joint_names, thetas)
                ]
                return JointConfiguration.create(positions).value
            jacobian = _jacobian_space(self._screw_axes, thetas)
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
