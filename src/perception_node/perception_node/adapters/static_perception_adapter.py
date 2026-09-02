"""Primer adaptador de `PerceptionPort` (shared_kernel): una `Scene` fija,
pasada en el constructor -- no detecta nada, solo cablea el puerto de punta
a punta para que `PlanningPort`/`PlannerSelectionPort` (controller_node)
tengan algo real que consumir mientras no existe percepción de verdad
(ROADMAP.md, Bloque 3: ground truth de CoppeliaSim primero, cámara real
después). Mismo papel que `NaiveTestKinematicsAdapter` tuvo para
`KinematicsPort`: no resuelve el problema, pero deja el puerto en pie.

Este adaptador en sí NO depende de rclpy (como el resto de adaptadores de
`PerceptionPort`, testeables sin ROS2) -- el nodo ROS2 real que publica su
`Scene` en `/perception/scene` vive en `perception_node/node.py`, aparte,
y es quien sí depende de `rclpy`/`ros2_kit`.
"""

from __future__ import annotations

from shared_kernel import Scene


class StaticPerceptionAdapter:
    def __init__(self, scene: Scene) -> None:
        self._scene = scene

    def get_scene(self) -> Scene:
        return self._scene
