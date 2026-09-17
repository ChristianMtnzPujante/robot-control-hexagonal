"""Cinemática vía álgebra geométrica conforme (CGA) con gafro/pygafro
(Löw, Abbet & Calinon, *GAFRO: Geometric Algebra for Robotics*, IEEE T-RO
2023 -- https://arxiv.org/abs/2212.07237).

Implementado el 17/09/2026 (F1.2 de la tesis) tras la prueba de viabilidad
F1.1 (`~/Desktop/doctorado/informe_F1_1_viabilidad_pygafro.md`): `pygafro`
se instala como rueda de PyPI (`pip install pygafro`), no hay nada que
compilar. Este módulo importa `pygafro` de forma perezosa (solo al construir
el adaptador) para que `controller_node` siga arrancando con las demás
estrategias aunque `pygafro` no esté instalado.

## La idea en cuatro líneas

En CGA (R^{4,1}) un movimiento rígido es un *motor* M = T·R (un versor:
traslación por rotación), y se aplica a cualquier primitiva x (punto,
línea, plano, esfera...) como x' = M x M~ -- la MISMA fórmula para todas.
Una articulación revoluta es un rotor R(θ) = exp(-θ/2 · B) con B el
bivector de su eje; una cadena serie es simplemente el producto de motores
M(θ) = Π_i (F_i · R_i(θ_i)), donde F_i es el motor fijo (origen del joint
en el URDF) que lleva del frame del joint i-1 al del joint i. Eso es
exactamente lo que `poe_adapter.py` calcula con matrices 4x4 y
exponenciales de se(3): mismos datos crudos de `RobotDescription`, otra
álgebra (ver docs/algebra_geometrica_conforme.md §4).

## Qué hace cada método

- `forward_kinematics`: M(θ) de la cadena completa (`System.
  computeKinematicChainMotor`), convertido a `Pose` (traslación + cuaternión).
- `link_poses`: producto acumulado de `joint.getMotor(θ_i)` -- la pose de
  CADA articulación, no solo del tip. Coincide con `PoeKinematicsAdapter.
  link_poses` (verificado en los tests, error < 1e-12).
- `compute_trajectory`: IK por Newton-Raphson amortiguado (Levenberg-
  Marquardt, mismo esquema y mismos parámetros por defecto que
  `poe_adapter.py`, para que la comparación PoE/GA sea justa) sobre el
  *logaritmo del motor de error* E = M_goal · M~(θ): `log(E)` es un
  bivector de 6 componentes (generador de motor) y el Jacobiano geométrico
  de gafro da, columna a columna, el generador que produce cada
  articulación -- ambos en la misma base de bivectores, así que el paso
  J^T (J J^T + λ²I)^{-1} log(E) se calcula sin salir del álgebra. Criterio
  de parada sobre el error geométrico real (distancia del tip y ángulo del
  rotor de error), no sobre las coordenadas del logaritmo: el logaritmo de
  un motor en gafro no es el twist de se(3) (coinciden en la parte
  rotacional y solo a primer orden en la traslacional), y el criterio de
  parada debe ser comparable al de PoE.

## Convenciones de pygafro (verificadas empíricamente contra Rodrigues, no
## documentadas upstream -- ver informe F1.1)

- Un `RotorGenerator` se construye con el vector [e12, e13, e23]; el eje
  URDF (x, y, z) corresponde a `RotorGenerator([z, -y, x])` (e12 = giro en
  torno a z, e23 = en torno a x, e13 = -giro en torno a y).
- Un `TranslatorGenerator([x, y, z])` es la traslación (x, y, z) sin
  reordenar; una prismática con eje (x, y, z) es `TranslatorGenerator([x,
  y, z])`.
- `Rotor.fromQuaternion` recibe y `Rotor.quaternion()` devuelve [w, x, y,
  z]; `Pose` del dominio usa (qx, qy, qz, qw).
- `Motor(Translator, Rotor)` es T·R: la matriz homogénea [R | t].
- Los `MotorGenerator` (Jacobiano y logaritmo) van en la base
  [e12, e13, e23, e1i, e2i, e3i]. Respecto del twist (w; v) de PoE:
  e12 = w_z, e13 = -w_y, e23 = w_x, (e1i, e2i, e3i) = v.

Lo que este adaptador NO hace todavía (y PoE tampoco): respetar límites
articulares, elegir entre las ramas de la IK, o usar la IK geométrica
cerrada de docs/algebra_geometrica_conforme.md §5. Lo que sí deja listo
para el resto de la tesis: el `System` de gafro (`self._system`) expone
Jacobianos de primitivas, matriz de masas y dinámica -- lo que hará falta
para la escena conforme (Fase 4a) y el MPC (Fase 4b).
"""

from __future__ import annotations

import math
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

from shared_kernel import (
    JointConfiguration,
    JointPosition,
    Pose,
    RobotDescription,
    Trajectory,
)

from .poe_adapter import _DEFAULT_CR5_DESCRIPTION

_CHAIN_NAME = "ee"

_PYGAFRO_MISSING = (
    "GaKinematicsAdapter necesita el paquete `pygafro` (Idiap, "
    "https://pypi.org/project/pygafro/): `python3 -m pip install --user "
    "pygafro`. Es una rueda precompilada, no hay que compilar nada -- ver "
    "~/Desktop/doctorado/informe_F1_1_viabilidad_pygafro.md."
)


def _require_pygafro() -> Any:
    try:
        import pygafro
    except ImportError as error:  # pragma: no cover - depende del entorno
        raise ImportError(_PYGAFRO_MISSING) from error
    return pygafro


def _rpy_to_quaternion_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Cuaternión [w, x, y, z] de R = Rz(yaw)·Ry(pitch)·Rx(roll) -- la misma
    convención rpy de URDF (ejes fijos) que `poe_adapter._rpy_to_matrix`."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def _joint_frame(ga: Any, xyz: Sequence[float], rpy: Sequence[float]) -> Any:
    """Motor fijo F_i = T·R del <origin> URDF de una articulación (el mismo
    que `gafro_robot_descriptions` construye al cargar un YAML)."""
    translator = ga.Translator(ga.TranslatorGenerator(np.asarray(xyz, dtype=float)))
    rotor = ga.Rotor.fromQuaternion(_rpy_to_quaternion_wxyz(*rpy))
    return ga.Motor(translator, rotor)


def _unit(axis: Sequence[float]) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    return vector / np.linalg.norm(vector)


def _build_system(ga: Any, description: RobotDescription) -> Any:
    """`pygafro.System` con un link por eslabón, una articulación por
    `JointDescription` (frame = origen URDF, eje = bivector/vector según el
    tipo) y una cadena cinemática `_CHAIN_NAME` que actúa todas en orden.
    Sin `Manipulator_N` (limitado a N<=10 en la rueda): la cadena explícita
    no tiene ese límite y expone lo mismo (`computeKinematicChainMotor`,
    `computeKinematicChainGeometricJacobian`)."""
    system = ga.System()
    system.setName(description.base_link)
    parent = system.createLink(description.base_link)
    last = len(description.joints) - 1
    for index, joint_description in enumerate(description.joints):
        child_name = description.tip_link if index == last else f"link_{joint_description.name}"
        child = system.createLink(child_name)
        if joint_description.joint_type == "prismatic":
            joint = system.createPrismaticJoint(joint_description.name)
            joint.setAxis(ga.TranslatorGenerator(_unit(joint_description.axis)))
        else:  # revolute / continuous
            x, y, z = _unit(joint_description.axis)
            joint = system.createRevoluteJoint(joint_description.name)
            joint.setAxis(ga.RotorGenerator(np.array([z, -y, x])))
        joint.setFrame(_joint_frame(ga, joint_description.origin_xyz, joint_description.origin_rpy))
        limits = ga.Joint.Limits()
        limits.positionLower, limits.positionUpper = -2 * math.pi, 2 * math.pi
        limits.velocity, limits.torque = float("inf"), float("inf")
        joint.setLimits(limits)
        joint.setParentLink(parent)
        parent.addChildJoint(joint)
        joint.setChildLink(child)
        child.setParentJoint(joint)
        parent = child
    system.finalize()
    chain = system.createKinematicChain(_CHAIN_NAME)
    for joint_description in description.joints:
        chain.addActuatedJoint(system.getJoint(joint_description.name))
    chain.finalize()
    return system


def _pose_to_motor(ga: Any, pose: Pose) -> Any:
    translator = ga.Translator(
        ga.TranslatorGenerator(np.array([pose.x, pose.y, pose.z], dtype=float))
    )
    quaternion = np.array([pose.qw, pose.qx, pose.qy, pose.qz], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        quaternion = quaternion / norm
    return ga.Motor(translator, ga.Rotor.fromQuaternion(quaternion))


def _motor_translation(motor: Any) -> np.ndarray:
    return np.asarray(motor.toTransformationMatrix())[:3, 3].copy()


def _motor_to_pose(motor: Any) -> Pose:
    x, y, z = _motor_translation(motor)
    qw, qx, qy, qz = np.asarray(motor.getRotor().quaternion()).ravel()
    return Pose(
        x=float(x), y=float(y), z=float(z),
        qx=float(qx), qy=float(qy), qz=float(qz), qw=float(qw),
    )


def _generator_vector(generator: Any) -> np.ndarray:
    """Coordenadas [e12, e13, e23, e1i, e2i, e3i] de un MotorGenerator."""
    return np.asarray(generator.vector(), dtype=float).ravel()


def _rotor_angle(motor: Any) -> float:
    """Ángulo de la rotación que contiene `motor` (0..π), a partir del
    escalar de su rotor: R = cos(θ/2) - sin(θ/2)·B."""
    scalar = float(motor.getRotor().get_scalar())
    return 2.0 * math.acos(min(1.0, abs(scalar)))


class GaKinematicsAdapter:
    """`KinematicsPort` sobre gafro/pygafro. Mismos parámetros y defaults
    que `PoeKinematicsAdapter` para que `strategy="ga"` y `strategy="poe"`
    sean intercambiables en `controller_node` y comparables entre sí."""

    def __init__(
        self,
        max_iterations: int = 200,
        orientation_tolerance: float = 1e-3,
        position_tolerance: float = 1e-4,
        damping_factor: float = 1e-2,
        steps: int = 20,
        robot_description: Optional[RobotDescription] = None,
    ) -> None:
        self._ga = _require_pygafro()
        description = robot_description or _DEFAULT_CR5_DESCRIPTION
        self._max_iterations = max_iterations
        self._orientation_tolerance = orientation_tolerance
        self._position_tolerance = position_tolerance
        self._damping_factor = damping_factor
        self._steps = steps
        self._joint_names: Tuple[str, ...] = description.joint_names
        self._system = _build_system(self._ga, description)
        self._joints = [self._system.getJoint(name) for name in self._joint_names]
        # Nº de iteraciones de la última IK -- solo diagnóstico (demos de
        # comparación PoE/GA), no forma parte de KinematicsPort.
        self.last_iteration_count: Optional[int] = None

    # -- KinematicsPort -----------------------------------------------------

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        target_configuration = self._inverse_kinematics(goal, current_configuration)
        return Trajectory.straight_line(
            current_configuration, target_configuration, self._steps
        )

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        """Pose del tip relativa a `base_link`: el motor de la cadena
        completa M(θ) = Π_i F_i·R_i(θ_i), como `Pose`."""
        return _motor_to_pose(self._ee_motor(self._thetas(configuration)))

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]:
        """Pose de CADA articulación (producto acumulado de motores hasta
        ella, incluida su propia rotación), en orden de cadena y en el
        mismo marco que `forward_kinematics`. `link_poses(...)[-1]` es el
        tip, como en PoE (no hay eslabón estático tras la última
        articulación en `RobotDescription`)."""
        thetas = self._thetas(configuration)
        motor = self._ga.Motor()
        poses = []
        for joint, theta in zip(self._joints, thetas):
            motor = self._ga.Motor(motor * joint.getMotor(float(theta)))
            poses.append(_motor_to_pose(motor))
        return poses

    # -- interno ------------------------------------------------------------

    def _thetas(self, configuration: JointConfiguration) -> np.ndarray:
        return np.array([configuration.angle_of(name) for name in self._joint_names])

    def _ee_motor(self, thetas: np.ndarray) -> Any:
        return self._system.computeKinematicChainMotor(_CHAIN_NAME, list(map(float, thetas)))

    def _geometric_jacobian(self, thetas: np.ndarray) -> np.ndarray:
        """6 x n: columna i = generador (bivector) que produce la
        articulación i en el frame de la base -- el equivalente CGA del
        Jacobiano espacial de PoE (misma información, base
        [e12, e13, e23, e1i, e2i, e3i])."""
        columns = self._system.computeKinematicChainGeometricJacobian(
            _CHAIN_NAME, list(map(float, thetas))
        )
        return np.column_stack([_generator_vector(column) for column in columns])

    def _inverse_kinematics(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> JointConfiguration:
        ga = self._ga
        goal_motor = _pose_to_motor(ga, goal)
        goal_position = _motor_translation(goal_motor)
        thetas = self._thetas(current_configuration)

        for iteration in range(self._max_iterations):
            current_motor = self._ee_motor(thetas)
            # Motor de error en el frame de la base: E·M(θ) = M_goal.
            error_motor = ga.Motor(goal_motor * current_motor.reverse())
            position_error = np.linalg.norm(goal_position - _motor_translation(current_motor))
            orientation_error = _rotor_angle(error_motor)
            if (
                orientation_error < self._orientation_tolerance
                and position_error < self._position_tolerance
            ):
                self.last_iteration_count = iteration
                positions = [
                    JointPosition(name, float(theta))
                    for name, theta in zip(self._joint_names, thetas)
                ]
                return JointConfiguration.create(positions).value
            error_generator = _generator_vector(error_motor.log().evaluate())
            jacobian = self._geometric_jacobian(thetas)
            damping_sq = self._damping_factor * float(error_generator @ error_generator)
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping_sq * np.eye(6), error_generator
            )
            thetas = thetas + step

        self.last_iteration_count = self._max_iterations
        raise RuntimeError(
            "GaKinematicsAdapter: Newton-Raphson no convergió en "
            f"{self._max_iterations} iteraciones para "
            f"Pose(x={goal.x}, y={goal.y}, z={goal.z}). "
            "Prueba un objetivo más cercano a la configuración actual del brazo."
        )
