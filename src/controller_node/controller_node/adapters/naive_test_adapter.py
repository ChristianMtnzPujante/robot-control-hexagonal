"""Doble de pruebas — NO es una estrategia de cinemática real.

Ignora por completo el objetivo cartesiano (Pose). Solo sirve para probar
el cableado Commander -> ControlSession -> controller_node -> robot_node
de extremo a extremo, sin depender de que PoE/GA/DH estén implementados.

No lo actives nunca como la estrategia "de verdad" en una sesión real.
"""

from __future__ import annotations

from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory


class NaiveTestKinematicsAdapter:
    def __init__(self, offset_radians: float = 0.1):
        self._offset_radians = offset_radians

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        del goal  # deliberadamente ignorado, ver docstring del módulo

        offset_positions = [
            JointPosition(p.joint_name, p.angle_radians + self._offset_radians)
            for p in current_configuration.positions
        ]
        offset_configuration = JointConfiguration.create(offset_positions).value

        result = Trajectory.create([current_configuration, offset_configuration])
        return result.value
