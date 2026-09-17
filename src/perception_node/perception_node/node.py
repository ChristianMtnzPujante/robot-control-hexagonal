"""El 'Nodo Perceptor': adaptador de entrada/salida ROS2 alrededor de un
`PerceptionPort`. Mismo papel que `robot_node`/`controller_node` para sus
respectivos puertos -- no decide nada, solo traduce (`to_scene_msg`) y
publica periódicamente lo que el adaptador concreto reporte.

A diferencia de `robot_node`/`controller_node`, este nodo NO vive dentro
del namespace de ninguna `ControlSession` -- tiene vida propia (spike de
ciclo de vida ya resuelto, Vikunja #89) y publica en un topic global,
`/perception/scene`, del que `Commander` escucha desde fuera de cualquier
sesión concreta (ver docs/nodos_ros2.md §4 y ROADMAP.md Bloque 3).

Con `perception_target="pseudo"` (`PseudoPerceptionAdapter`), este nodo
además ESCUCHA dos topics de entrada, `/perception/report_obstacle` y
`/perception/report_object` -- así un proceso externo (un demo, `ros2
topic pub`, o en el futuro un detector real) puede "inyectar" eventos sin
tener que compartir el mismo proceso Python que este nodo, que era la
única forma de usar `PseudoPerceptionAdapter` hasta ahora.
"""

from __future__ import annotations

from rclpy.node import Node
from ros2_kit import (
    apply_node_config,
    from_obstacle_report_msg,
    from_object_report_msg,
    load_node_config,
    package_config_path,
    run_node,
    to_scene_msg,
)
from shared_kernel import Scene

from .adapters.file_perception_adapter import FilePerceptionAdapter
from .adapters.pseudo_perception_adapter import PseudoPerceptionAdapter
from .adapters.static_perception_adapter import StaticPerceptionAdapter

_SCENE_TOPIC = "/perception/scene"
_OBSTACLE_REPORT_TOPIC = "/perception/report_obstacle"
_OBJECT_REPORT_TOPIC = "/perception/report_object"
_CONFIG_PATH = package_config_path("perception_node", "perception_node.yaml")


class PerceptionNode(Node):
    def __init__(self) -> None:
        config = load_node_config(_CONFIG_PATH)
        super().__init__(config.node_name)
        self._topic_publishers = apply_node_config(self, config)

        target = self.get_parameter("perception_target").value
        file_path = self.get_parameter("file_path").value
        self._perception = self._build_adapter(target, file_path)

        period = float(self.get_parameter("scene_publish_period_seconds").value)
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
        if target == "pseudo":
            return PseudoPerceptionAdapter()
        raise ValueError(f'perception_target desconocido: "{target}"')

    def _publish_scene(self) -> None:
        scene = self._perception.get_scene()
        self._topic_publishers[_SCENE_TOPIC].publish(to_scene_msg(scene))

    def _on_report_obstacle(self, msg) -> None:
        # Duck typing deliberado (getattr, no isinstance): el mismo
        # callback está suscrito sin importar qué perception_target esté
        # activo, porque las subscriptions se crean desde el YAML antes de
        # saber el target (ver apply_node_config) -- solo PseudoPerceptionAdapter
        # sabe qué hacer con esto hoy.
        report_obstacle = getattr(self._perception, "report_obstacle", None)
        if report_obstacle is None:
            self.get_logger().warning(
                f'"{_OBSTACLE_REPORT_TOPIC}" recibido pero perception_target='
                f'"{self.get_parameter("perception_target").value}" no admite '
                "inyección de eventos (usa perception_target=\"pseudo\")"
            )
            return
        name, obstacle = from_obstacle_report_msg(msg)
        report_obstacle(name, obstacle)

    def _on_report_object(self, msg) -> None:
        report_object = getattr(self._perception, "report_object", None)
        if report_object is None:
            self.get_logger().warning(
                f'"{_OBJECT_REPORT_TOPIC}" recibido pero perception_target='
                f'"{self.get_parameter("perception_target").value}" no admite '
                "inyección de eventos (usa perception_target=\"pseudo\")"
            )
            return
        name, position = from_object_report_msg(msg)
        report_object(name, position)


def main(args=None):
    run_node(PerceptionNode, args=args)


if __name__ == "__main__":
    main()
