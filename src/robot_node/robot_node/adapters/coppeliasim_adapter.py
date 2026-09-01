"""Adaptador de salida (driven adapter): habla con CoppeliaSim vía su
API remota ZMQ, directamente desde este proceso Python — sin pasar por
ningún script Lua embebido en la escena.

Implementa RobotControllerPort por duck typing (typing.Protocol): no
hereda de nada, solo tiene los métodos que el puerto exige.

Además decora la escena para poder ver la trayectoria: si se le da
`tip_name`, deja un dummy en cada waypoint por el que pasa (trail); y si
algo le manda el objetivo cartesiano vía `mark_goal`, otro dummy distinto
en el punto objetivo. Ninguna de las dos cosas forma parte de
RobotControllerPort -- son puramente cosméticas y específicas de este
adaptador, así que robot_node las usa solo si el adaptador las ofrece.

Si se le da `scene_path`, carga esa escena y arranca la simulación él
mismo (en vez de exigir que alguien tenga CoppeliaSim ya abierto con la
escena correcta y en play a mano). Sin `scene_path`, el comportamiento es
el de siempre: asume que la escena ya está cargada y en play.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shared_kernel import JointConfiguration, JointPosition, Pose, SphereObstacle

_GOAL_COLOR = [1.0, 0.0, 0.0] * 4  # rojo: objetivo cartesiano
_TRAIL_COLOR = [0.0, 0.4, 1.0] * 4  # azul: waypoints intermedios
_OBSTACLE_COLOR = [1.0, 0.55, 0.0]  # naranja: obstáculo a evitar


class CoppeliaSimRobotAdapter:
    def __init__(
        self,
        joint_names: List[str],
        tip_name: Optional[str] = None,
        scene_path: Optional[str] = None,
        zmq_port: int = 23000,
    ):
        self._client = RemoteAPIClient(port=zmq_port)
        self._sim = self._client.require("sim")
        if scene_path:
            self._load_scene_and_play(scene_path)
        self._joint_names = list(joint_names)
        self._handles: Dict[str, int] = {
            name: self._sim.getObject(f"/{name}") for name in self._joint_names
        }
        self._tip_handle = self._sim.getObject(f"/{tip_name}") if tip_name else None
        self._goal_dummy_handle: Optional[int] = None
        self._waypoint_count = 0

    def _load_scene_and_play(self, scene_path: str) -> None:
        # loadScene exige la simulación parada -- stopSimulation es
        # idempotente si ya lo estaba, así que no hace falta comprobar
        # el estado antes de llamarla.
        self._sim.stopSimulation()
        while self._sim.getSimulationState() != self._sim.simulation_stopped:
            time.sleep(0.1)
        self._sim.loadScene(scene_path)
        self._sim.startSimulation()

    def set_joints(self, configuration: JointConfiguration) -> None:
        for position in configuration.positions:
            handle = self._handles[position.joint_name]
            self._sim.setJointPosition(handle, position.angle_radians)
        self._drop_trail_marker()

    def get_current_configuration(self) -> JointConfiguration:
        positions = [
            JointPosition(name, self._sim.getJointPosition(handle))
            for name, handle in self._handles.items()
        ]
        result = JointConfiguration.create(positions)
        # positions nunca está vacío aquí (viene de self._joint_names, ya validado
        # al construir el adaptador), así que este Either siempre es Right.
        return result.value

    def mark_goal(self, goal: Pose) -> None:
        if self._goal_dummy_handle is None:
            self._goal_dummy_handle = self._sim.createDummy(0.03, _GOAL_COLOR)
            self._sim.setObjectAlias(self._goal_dummy_handle, "objetivo")
        self._sim.setObjectPosition(
            self._goal_dummy_handle, -1, [goal.x, goal.y, goal.z]
        )

    def mark_obstacle(self, obstacle: SphereObstacle) -> None:
        """Cosmético, igual que `mark_goal`: no forma parte de
        `RobotControllerPort`, solo ayuda a ver en la escena lo que un
        `PlanningPort` está evitando (ver
        `controller_node/adapters/obstacle_avoiding_planning_adapter.py`).
        Crea una esfera visual real del radio del obstáculo -- sin física
        (options=0: no se marca dynamic ni no-respondable a propósito,
        puramente decorativa mientras nada del sistema lea colisiones de
        verdad).

        MISMA limitación de frame que `mark_goal`: `setObjectPosition` con
        parent=-1 coloca en coordenadas de MUNDO, mientras que `Pose`/
        `SphereObstacle` viven en el marco interno de `KinematicsPort` (ver
        `PoeKinematicsAdapter`), que solo coincide con el de CoppeliaSim si
        el robot está en el origen del mundo con orientación identidad (ver
        `commander/coppeliasim_scene_builder.py`, que lo garantiza al
        importar desde URDF sin recentrar) -- con una escena hecha a mano
        donde el robot esté en otra pose, habría que transformar antes de
        llamar a `mark_goal`/`mark_obstacle` (ver el hallazgo, ya resuelto
        para este repo, que documentaba `two_sessions_demo.py`)."""
        handle = self._sim.createPrimitiveShape(
            self._sim.primitiveshape_spheroid, [obstacle.radius * 2] * 3
        )
        self._sim.setShapeColor(
            handle, "", self._sim.colorcomponent_ambient_diffuse, _OBSTACLE_COLOR
        )
        self._sim.setObjectAlias(handle, "obstaculo")
        self._sim.setObjectPosition(
            handle, -1, [obstacle.center.x, obstacle.center.y, obstacle.center.z]
        )

    def _drop_trail_marker(self) -> None:
        # Sin tip_name no sabemos qué objeto leer para la posición cartesiana
        # del waypoint, así que no decoramos nada (RobotControllerPort sigue
        # cumpliéndose igual, esto es solo cosmético).
        if self._tip_handle is None:
            return
        position = self._sim.getObjectPosition(self._tip_handle, -1)
        handle = self._sim.createDummy(0.015, _TRAIL_COLOR)
        self._waypoint_count += 1
        self._sim.setObjectAlias(handle, f"waypoint_{self._waypoint_count}")
        self._sim.setObjectPosition(handle, -1, position)
