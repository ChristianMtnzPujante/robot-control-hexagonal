"""Formato de un TIPO DE NODO (`src/<paquete>/config/<nodo>.yaml`): la
interfaz de un nodo -- sus parámetros, qué publica, a qué se suscribe y
sus timers. Lo lee `ros2_kit.load_node_config` al arrancar el nodo; aquí
solo se declara su esquema, para la guía y, en el siguiente paso, para el
grafo de nodos de una célula (quién publica qué a quién).

No es parte de `descriptions/`: cada nodo trae el suyo en su paquete,
porque es la "clase" del nodo, no una instancia en una célula.
"""

from __future__ import annotations

from .schema import Field, Section

NODE_TYPE = Section(
    "Tipo de nodo",
    "src/<paquete>/config/<nodo>.yaml",
    (
        Field("node", "`{name}`", True, "Nombre del nodo."),
        Field("parameters", "dict nombre → parámetro", False, "Parámetros declarados (ver «Tipo de nodo: parámetro»)."),
        Field("publishers", "lista de topics", False, "Lo que publica (ver «Tipo de nodo: topic»)."),
        Field("subscriptions", "lista de topics", False, "A qué se suscribe (ver «Tipo de nodo: topic»)."),
        Field("timers", "lista", False, "`period_parameter` (nombre de un parámetro) + `callback` (método del nodo)."),
    ),
    "La lógica nunca va aquí: `callback` es solo el NOMBRE del método que atiende el canal.",
)

NODE_PARAMETER = Section(
    "Tipo de nodo: parámetro",
    "node: parameters.<nombre>",
    (
        Field("type", "`string` | `int` | `double` | `bool` | `string_array` | `double_array`", True, "Tipo ROS 2 del parámetro."),
        Field("default", "valor", True, "Valor por defecto."),
        Field("range", "`{min, max}`", False, "Solo `int`/`double`: rclpy rechaza en ejecución lo que quede fuera."),
    ),
)

NODE_TOPIC = Section(
    "Tipo de nodo: topic",
    "node: publishers[i] / subscriptions[i]",
    (
        Field("topic", "nombre", True, "Relativo (`goal`): dentro del namespace de la instancia. Absoluto (`/perception/scene`): global."),
        Field("message_type", "`paquete/msg/Tipo`", True, "Tipo de mensaje, p. ej. `sensor_msgs/msg/JointState`."),
        Field("qos", "entero o perfil", True, "Profundidad de cola (entero) o un perfil de `ros2_kit.qos`: `GOAL_QOS`, `SCENE_QOS`, `STRATEGY_QOS`."),
        Field("callback", "nombre de método", "en suscripciones", "Método del nodo que atiende los mensajes."),
    ),
)
