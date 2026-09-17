"""Doble de pruebas — NO es una estrategia de cinemática real.

Ignora por completo el objetivo cartesiano (Pose). Solo sirve para probar
el cableado Commander -> ControlSession -> controller_node -> robot_node
de extremo a extremo, sin depender de que PoE/GA/DH estén implementados.

Genera un barrido sinusoidal (varios waypoints, no un único salto) para
que el movimiento se note visualmente a través del propio pipeline.

No lo actives nunca como la estrategia "de verdad" en una sesión real.
"""

from __future__ import annotations

import math
from typing import List

from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory


class NaiveTestKinematicsAdapter:
    def __init__(self, amplitude_radians: float = 0.8, steps: int = 20):
        self._amplitude_radians = amplitude_radians
        self._steps = steps

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        del goal  # deliberadamente ignorado, ver docstring del módulo

        waypoints = []
        for i in range(self._steps + 1):
            phase = (i / self._steps) * 2 * math.pi  # un ciclo completo: ida y vuelta
            offset = self._amplitude_radians * math.sin(phase)
            positions = [
                JointPosition(p.joint_name, p.angle_radians + offset)
                for p in current_configuration.positions
            ]
            waypoints.append(JointConfiguration.create(positions).value)

        return Trajectory.create(waypoints).value

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        # Parte formal de KinematicsPort desde el 08/09 (ver shared_kernel/
        # ports.py). Este doble no tiene ningún modelo geométrico del robot
        # -- devolver una Pose inventada sería más peligroso que fallar
        # ruidosamente (nadie que use este doble para probar el cableado
        # debería estar apoyándose además en su geometría, ver docstring
        # del módulo: "no lo actives nunca como estrategia de verdad").
        raise NotImplementedError(
            "NaiveTestKinematicsAdapter: doble de pruebas, no implementa "
            "cinemática directa real -- no lo uses con un PlanningPort que "
            "la necesite (ver ObstacleAvoidingPlanningAdapter)."
        )

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]:
        raise NotImplementedError(
            "NaiveTestKinematicsAdapter: doble de pruebas, no implementa "
            "cinemática directa real -- no lo uses con un PlanningPort que "
            "la necesite (ver WholeBodyObstacleAvoidingPlanningAdapter)."
        )
