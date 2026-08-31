"""Primer adaptador de `PlanningPort` (shared_kernel): envuelve un
`KinematicsPort` ya existente e ignora la `Scene` recibida por completo --
NO evita obstáculos, es la asignación más simple posible que satisface el
puerto. Mismo papel que `NaiveTestKinematicsAdapter` tuvo para
`KinematicsPort`: deja `PlanningPort` cableado de punta a punta (algo real
que probar) antes de invertir en CHOMP/RRT (ROADMAP.md, Bloque 4).

Cuando llegue un planificador real, la `Scene` sí importará -- por eso el
parámetro ya está en la firma en vez de añadirse después.
"""

from __future__ import annotations

from shared_kernel import JointConfiguration, KinematicsPort, Pose, Scene, Trajectory


class NaivePlanningAdapter:
    def __init__(self, kinematics: KinematicsPort) -> None:
        self._kinematics = kinematics

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration, scene: Scene
    ) -> Trajectory:
        return self._kinematics.compute_trajectory(goal, current_configuration)
