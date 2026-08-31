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
        waypoint_period_seconds: float = 0.5,
        tip_name: str = "",
        scene_path: str = "",
        zmq_port: int = 23000,
        urdf_path: str = "",
        base_link: str = "",
        tip_link: str = "",
    ):
        self.namespace = namespace
        self._robot_target = robot_target
        self._controller_strategy = controller_strategy
        self._joint_names = joint_names
        self._waypoint_period_seconds = waypoint_period_seconds
        self._tip_name = tip_name
        self._scene_path = scene_path
        self._zmq_port = zmq_port
        # Geometría real del robot (URDF), solo relevante para controller_node
        # (cinemática) -- robot_node no la necesita. Vacío conserva el CR5
        # hardcodeado por defecto en cada KinematicsPort (ver ROADMAP.md,
        # Bloque 9).
        self._urdf_path = urdf_path
        self._base_link = base_link
        self._tip_link = tip_link
        self._robot_process: Optional[subprocess.Popen] = None
        self._controller_process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        joint_names_yaml = "[" + ",".join(self._joint_names) + "]"

        robot_args = [
            "ros2", "run", "robot_node", "robot_node",
            "--ros-args",
            "-r", f"__ns:={self.namespace}",
            "-p", f"robot_target:={self._robot_target}",
            "-p", f"joint_names:={joint_names_yaml}",
            "-p", f"zmq_port:={self._zmq_port}",
        ]
        controller_args = [
            "ros2", "run", "controller_node", "controller_node",
            "--ros-args",
            "-r", f"__ns:={self.namespace}",
            "-p", f"strategy:={self._controller_strategy}",
            "-p", f"waypoint_period_seconds:={self._waypoint_period_seconds}",
            "-p", f"zmq_port:={self._zmq_port}",
            "-p", f"joint_names:={joint_names_yaml}",
        ]
        # -p x:= con valor vacío rompe el parseo de argumentos de ROS2
        # ("Couldn't parse parameter override rule") -- se omiten del todo
        # en vez de mandarlos vacíos, dejando que declare_parameter use su
        # propio default ("").
        if self._tip_name:
            robot_args += ["-p", f"tip_name:={self._tip_name}"]
            controller_args += ["-p", f"tip_name:={self._tip_name}"]
        if self._scene_path:
            robot_args += ["-p", f"scene_path:={self._scene_path}"]
            controller_args += ["-p", f"scene_path:={self._scene_path}"]
        if self._urdf_path:
            controller_args += ["-p", f"urdf_path:={self._urdf_path}"]
        if self._base_link:
            controller_args += ["-p", f"base_link:={self._base_link}"]
        if self._tip_link:
            controller_args += ["-p", f"tip_link:={self._tip_link}"]
        self._robot_process = subprocess.Popen(robot_args)
        self._controller_process = subprocess.Popen(controller_args)

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
