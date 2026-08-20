"""Errores de dominio. No dependen de ROS2 ni de ningún framework."""


class InvalidJointPositionError(Exception):
    pass


class EmptyJointConfigurationError(Exception):
    def __init__(self) -> None:
        super().__init__(
            "Una configuración de articulaciones no puede estar vacía"
        )


class EmptyTrajectoryError(Exception):
    def __init__(self) -> None:
        super().__init__(
            "Una trayectoria no puede estar vacía, necesita al menos un waypoint"
        )


class TrajectoryVerificationFailedError(Exception):
    pass


class InvalidRobotDescriptionError(Exception):
    pass
