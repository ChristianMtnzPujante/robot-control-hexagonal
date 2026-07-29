"""Cinemática vía Product of Exponentials (Explicit-Robotics, Nah & Lachner).

TODO — pendiente de verdad: falta extraer los twists S_i=(w_i, v_i) del CR5
en su configuración home (desde el URDF ya importado en
~/RoboticInvest/CoppeliaSim/scenes/cr5_base.ttt) e implementar el bucle
Newton-Raphson con el Jacobiano vía la representación Adjunta, tal como
se dedujo a mano en las sesiones de teoría.
"""

from __future__ import annotations

from shared_kernel import JointConfiguration, Pose, Trajectory


class PoeKinematicsAdapter:
    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        raise NotImplementedError(
            "PoeKinematicsAdapter: falta implementar el producto de "
            "exponenciales + Newton-Raphson vía Adjunta."
        )
