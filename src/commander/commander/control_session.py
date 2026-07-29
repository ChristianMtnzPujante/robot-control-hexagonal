"""ControlSession: empareja UN controlador (cálculo) con UN robot
(ejecución) durante un periodo acotado, en su propio namespace ROS2.

El 'cableado' de la relación entre ambos nodos es, literalmente, que
comparten namespace: controller_node publica en <ns>/joint_command y
robot_node se suscribe ahí; robot_node publica en <ns>/joint_states y
controller_node se suscribe ahí. Ninguno de los dos necesita saber
nada del otro más allá de ese namespace común.

Cada nodo es un PROCESO aparte de verdad (subprocess), no un hilo dentro
del proceso del Commander -- así se puede lanzar sim/real por separado,
incluso en máquinas distintas si algún día hace falta.
"""

from __future__ import annotations

import subprocess
from typing import List, Optional


class ControlSession:
    def __init__(
        self,
        namespace: str,
        robot_target: str,
        controller_strategy: str,
        joint_names: List[str],
    ):
        self.namespace = namespace
        self._robot_target = robot_target
        self._controller_strategy = controller_strategy
        self._joint_names = joint_names
        self._robot_process: Optional[subprocess.Popen] = None
        self._controller_process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        joint_names_yaml = "[" + ",".join(self._joint_names) + "]"

        self._robot_process = subprocess.Popen(
            [
                "ros2", "run", "robot_node", "robot_node",
                "--ros-args",
                "-r", f"__ns:={self.namespace}",
                "-p", f"robot_target:={self._robot_target}",
                "-p", f"joint_names:={joint_names_yaml}",
            ]
        )
        self._controller_process = subprocess.Popen(
            [
                "ros2", "run", "controller_node", "controller_node",
                "--ros-args",
                "-r", f"__ns:={self.namespace}",
                "-p", f"strategy:={self._controller_strategy}",
            ]
        )

    def stop(self) -> None:
        for process in (self._robot_process, self._controller_process):
            if process is None:
                continue
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def __enter__(self) -> "ControlSession":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()
