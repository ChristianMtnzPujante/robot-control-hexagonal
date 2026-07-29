from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .either import Either, left, right
from .errors import EmptyTrajectoryError
from .value_objects import JointConfiguration


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
