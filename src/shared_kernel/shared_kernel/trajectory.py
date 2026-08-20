from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .either import Either, left, right
from .errors import EmptyTrajectoryError
from .value_objects import JointConfiguration, JointPosition


@dataclass(frozen=True)
class Trajectory:
    """Una secuencia ordenada de waypoints a ejecutar en orden.

    Es lo que produce un KinematicsPort (PoE, GA, DH...) y lo que
    consume un RobotControllerPort, waypoint a waypoint.
    """

    waypoints: List[JointConfiguration]

    @staticmethod
    def create(
        waypoints: List[JointConfiguration],
    ) -> Either[EmptyTrajectoryError, "Trajectory"]:
        if not waypoints:
            return left(EmptyTrajectoryError())
        return right(Trajectory(list(waypoints)))

    @staticmethod
    def straight_line(
        start: JointConfiguration, end: JointConfiguration, steps: int
    ) -> "Trajectory":
        """Interpola linealmente en espacio de articulaciones entre `start`
        y `end` (una recta, pero en ángulos, no en el espacio cartesiano).

        Cualquier KinematicsPort que ya tenga una JointConfiguration objetivo
        (vía IK, sea cual sea el método) puede apoyarse en esto en vez de
        reimplementar la interpolación.
        """
        joint_names = [p.joint_name for p in start.positions]
        waypoints = []
        for i in range(steps + 1):
            t = i / steps
            positions = [
                JointPosition(
                    name,
                    start.angle_of(name)
                    + t * (end.angle_of(name) - start.angle_of(name)),
                )
                for name in joint_names
            ]
            waypoints.append(JointConfiguration.create(positions).value)
        # steps >= 1 garantiza al menos [start, end]; nunca vacío.
        return Trajectory.create(waypoints).value
