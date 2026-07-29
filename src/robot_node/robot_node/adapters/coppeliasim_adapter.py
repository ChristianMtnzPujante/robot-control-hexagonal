"""Adaptador de salida (driven adapter): habla con CoppeliaSim vía su
API remota ZMQ, directamente desde este proceso Python — sin pasar por
ningún script Lua embebido en la escena.

Implementa RobotControllerPort por duck typing (typing.Protocol): no
hereda de nada, solo tiene los métodos que el puerto exige.
"""

from __future__ import annotations

from typing import Dict, List

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shared_kernel import JointConfiguration, JointPosition


class CoppeliaSimRobotAdapter:
    def __init__(self, joint_names: List[str]):
        self._client = RemoteAPIClient()
        self._sim = self._client.require("sim")
        self._joint_names = list(joint_names)
        self._handles: Dict[str, int] = {
            name: self._sim.getObject(f"/{name}") for name in self._joint_names
        }

    def set_joints(self, configuration: JointConfiguration) -> None:
        for position in configuration.positions:
            handle = self._handles[position.joint_name]
            self._sim.setJointPosition(handle, position.angle_radians)

    def get_current_configuration(self) -> JointConfiguration:
        positions = [
            JointPosition(name, self._sim.getJointPosition(handle))
            for name, handle in self._handles.items()
        ]
        result = JointConfiguration.create(positions)
        # positions nunca está vacío aquí (viene de self._joint_names, ya validado
        # al construir el adaptador), así que este Either siempre es Right.
        return result.value
