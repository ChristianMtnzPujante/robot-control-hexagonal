"""Recta en espacio de articulaciones + IK aproximada — NO es una estrategia
de cinemática real (como naive_test), pero a diferencia de ese doble de
pruebas, SÍ tiene en cuenta el objetivo: sirve para probar el pipeline
completo (objetivo cartesiano -> una configuración que lo "alcanza" ->
interpolación recta) sin depender todavía de la matemática real de
PoE/GA/DH.

`_approximate_inverse_kinematics` NO es la cinemática inversa real del CR5:
es una asignación determinista y arbitraria de Pose -> JointConfiguration,
solo para tener un objetivo plausible con el que trazar una recta y ver
algo moverse en Coppelia. No usar como referencia física ni en producción.
"""

from __future__ import annotations

import math
from typing import List

from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory


class StraightLineKinematicsAdapter:
    def __init__(self, steps: int = 20):
        self._steps = steps

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        target_configuration = self._approximate_inverse_kinematics(
            goal, current_configuration
        )
        return Trajectory.straight_line(
            current_configuration, target_configuration, self._steps
        )

    def _approximate_inverse_kinematics(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> JointConfiguration:
        joint_names = [p.joint_name for p in current_configuration.positions]
        base_yaw = math.atan2(goal.y, goal.x)
        radius = math.hypot(goal.x, goal.y)
        angles = [
            base_yaw,
            radius,
            goal.z,
            math.atan2(goal.qz, goal.qw),
            radius - goal.z,
            base_yaw + goal.z,
        ]
        positions = [
            JointPosition(name, angles[i % len(angles)])
            for i, name in enumerate(joint_names)
        ]
        return JointConfiguration.create(positions).value

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        # Parte formal de KinematicsPort desde el 08/09 (ver shared_kernel/
        # ports.py). `_approximate_inverse_kinematics` es una asignación
        # arbitraria Pose->JointConfiguration sin geometría real detrás --
        # no existe una "inversa" que deshacer, así que no hay forma
        # honesta de dar cinemática directa aquí (ver docstring del
        # módulo: no usar como referencia física).
        raise NotImplementedError(
            "StraightLineKinematicsAdapter: no tiene un modelo geométrico "
            "real, no implementa cinemática directa -- no lo uses con un "
            "PlanningPort que la necesite (ver ObstacleAvoidingPlanningAdapter)."
        )

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]:
        raise NotImplementedError(
            "StraightLineKinematicsAdapter: no tiene un modelo geométrico "
            "real, no implementa cinemática directa -- no lo uses con un "
            "PlanningPort que la necesite (ver WholeBodyObstacleAvoidingPlanningAdapter)."
        )
