"""El 'Nodo Perceptor': adaptador de entrada/salida ROS2 alrededor de un
`PerceptionPort`. Mismo papel que `robot_node`/`controller_node` para sus
respectivos puertos -- no decide nada, solo traduce (`to_scene_msg`) y
publica periódicamente lo que el adaptador concreto reporte.

A diferencia de `robot_node`/`controller_node`, este nodo NO vive dentro
del namespace de ninguna `ControlSession` -- tiene vida propia (spike de
ciclo de vida ya resuelto, Vikunja #89) y publica en un topic global,
`/perception/scene`, del que `Commander` escucha desde fuera de cualquier
sesión concreta (ver docs/nodos_ros2.md §4 y ROADMAP.md Bloque 3).
"""

from __future__ import annotations

from rclpy.node import Node
from ros2_kit import SCENE_QOS, run_node, to_scene_msg
from shared_kernel import Scene
from std_msgs.msg import String

from .adapters.file_perception_adapter import FilePerceptionAdapter
from .adapters.static_perception_adapter import StaticPerceptionAdapter

_SCENE_TOPIC = "/perception/scene"


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("perception_target", "fichero")
        self.declare_parameter("file_path", "")
        self.declare_parameter("scene_publish_period_seconds", 0.5)

        target = self.get_parameter("perception_target").value
        file_path = self.get_parameter("file_path").value
        self._perception = self._build_adapter(target, file_path)

        self._scene_pub = self.create_publisher(String, _SCENE_TOPIC, SCENE_QOS)

        period = float(self.get_parameter("scene_publish_period_seconds").value)
        self._timer = self.create_timer(period, self._publish_scene)

        self.get_logger().info(
            f'perception_node listo, target="{target}", '
            f"publicando en {_SCENE_TOPIC} cada {period}s"
        )

    def _build_adapter(self, target: str, file_path: str):
        if target == "fichero":
            if not file_path:
                raise ValueError('perception_target="fichero" exige "file_path"')
            return FilePerceptionAdapter(file_path)
        if target == "estatico":
            return StaticPerceptionAdapter(Scene.empty())
        raise ValueError(f'perception_target desconocido: "{target}"')

    def _publish_scene(self) -> None:
        scene = self._perception.get_scene()
        self._scene_pub.publish(to_scene_msg(scene))


def main(args=None):
    run_node(PerceptionNode, args=args)


if __name__ == "__main__":
    main()
