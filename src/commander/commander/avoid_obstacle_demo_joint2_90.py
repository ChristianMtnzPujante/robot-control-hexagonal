"""Segundo ejemplo de `avoid_obstacle_demo`: misma orquestación
(`avoid_obstacle_demo.run`), distinta "descripción" -- postura inicial CON
`joint2` a 90° en vez de la posición base (todos a cero), y un
goal/obstáculo distintos. Demuestra que cambiar de ejemplo es cuestión de
la descripción (postura inicial + `Scene`), no de tocar la orquestación.

Postura inicial: `joint2 = 90°` (`math.pi / 2`), resto a cero.
Pose real de esa postura (vía `PoeKinematicsAdapter.forward_kinematics`):
(-0.900, -0.246, 0.147).

Goal: mismo eje de orientación que la postura inicial, movido a
(-0.30, 0.35, 0.25) -- convergió numéricamente (distancia ~0.85 m,
comprobado antes de fijarlo aquí).

Obstáculo: esfera de radio 0.08 m centrada en el punto medio real de ese
segmento, (-0.600, 0.052, 0.199).

Uso: `ros2 run commander avoid_obstacle_demo_joint2_90`.
"""

from __future__ import annotations

import math

from shared_kernel import JointConfiguration, JointPosition, Point, Pose, SphereObstacle

from .avoid_obstacle_demo import run

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

_INITIAL_CONFIGURATION = JointConfiguration.create(
    [
        JointPosition(name, math.pi / 2 if name == "joint2" else 0.0)
        for name in _JOINT_NAMES
    ]
).value

_GOAL = Pose(
    x=-0.30, y=0.35, z=0.25,
    qx=-0.5, qy=-0.5, qz=0.5, qw=-0.5,
)
_OBSTACLE = SphereObstacle(center=Point(-0.600, 0.052, 0.199), radius=0.08)


def main() -> None:
    run(_INITIAL_CONFIGURATION, _GOAL, _OBSTACLE)


if __name__ == "__main__":
    main()
