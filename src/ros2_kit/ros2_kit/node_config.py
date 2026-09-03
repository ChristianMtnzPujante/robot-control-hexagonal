"""Config declarativa de nodo (parámetros + publishers + subscriptions +
timers) en un YAML aparte, en vez de repetido a mano en cada `__init__` de
`robot_node`/`controller_node`/`perception_node`.

Dos funciones, dos responsabilidades que NO se mezclan:

- `load_node_config(path)`: solo lee el YAML y resuelve nombres a objetos
  reales (el string "pkg/msg/Tipo" -> la clase de mensaje, el nombre de QoS
  -> el QoSProfile de `ros2_kit.qos`). NO toca `rclpy` -- puede llamarse
  ANTES de que exista el `Node`, porque hace falta el nombre del propio YAML
  para poder construirlo (`super().__init__(config.node_name)`).

- `apply_node_config(node, config)`: al revés, solo llama a `rclpy` de
  verdad (`declare_parameter`, `create_publisher`, `create_subscription`,
  `create_timer`) sobre un `node` YA construido -- estos métodos no existen
  hasta que `Node.__init__` ha terminado.

Lo que el YAML NUNCA contiene es lógica: el campo `callback` de una
subscription/timer es solo el NOMBRE del método (`"_on_goal"`), resuelto con
`getattr(node, nombre)` en `apply_node_config`. El cuerpo de ese método
sigue definido, como siempre, en la clase del nodo -- la config solo dice
"qué método atiende qué canal", nunca "qué hace ese método".
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from .qos import GOAL_QOS, SCENE_QOS, STRATEGY_QOS

# Registro nombre -> QoSProfile: el YAML referencia un perfil por nombre
# (p.ej. "GOAL_QOS"), nunca redefine reliability/durability -- ese es un
# criterio de diseño (ver docstring de ros2_kit/qos.py) que sigue viviendo
# en código, no en datos. Un `qos:` numérico en el YAML (p.ej. `qos: 10`) se
# trata aparte, como profundidad de cola con el resto de la QoS por defecto
# -- exactamente lo que hoy hace `create_publisher(Tipo, topic, 10)`.
_QOS_BY_NAME: Dict[str, Any] = {
    "GOAL_QOS": GOAL_QOS,
    "STRATEGY_QOS": STRATEGY_QOS,
    "SCENE_QOS": SCENE_QOS,
}


def _resolve_qos(value: Union[int, str]):
    if isinstance(value, int):
        return value  # profundidad de cola simple, mismo significado que hoy
    try:
        return _QOS_BY_NAME[value]
    except KeyError:
        raise ValueError(
            f'qos "{value}" desconocida -- perfiles disponibles: '
            f"{sorted(_QOS_BY_NAME)} (o un entero para profundidad simple)"
        )


@dataclass
class ParameterSpec:
    name: str
    default: Any
    descriptor: ParameterDescriptor


@dataclass
class TopicSpec:
    topic: str
    message_type: type  # ya resuelta -- la clase de mensaje, no el string
    qos: Any
    callback: Optional[str] = None  # solo aplica a subscriptions


@dataclass
class TimerSpec:
    period_parameter: str  # nombre de un ParameterSpec ya declarado
    callback: str


@dataclass
class NodeConfig:
    node_name: str
    namespace: str = ""
    parameters: List[ParameterSpec] = field(default_factory=list)
    publishers: List[TopicSpec] = field(default_factory=list)
    subscriptions: List[TopicSpec] = field(default_factory=list)
    timers: List[TimerSpec] = field(default_factory=list)


def _build_descriptor(spec: dict) -> ParameterDescriptor:
    # Traduce "range" (si lo hay) a un ParameterDescriptor real -- esto hace
    # que rclpy RECHACE en runtime cualquier valor fuera de rango, no es solo
    # documentación (ver rcl_interfaces/msg/ParameterDescriptor.msg).
    descriptor = ParameterDescriptor()
    range_spec = spec.get("range")
    if range_spec is None:
        return descriptor
    parameter_type = spec.get("type")
    if parameter_type == "double":
        descriptor.floating_point_range = [
            FloatingPointRange(from_value=float(range_spec["min"]), to_value=float(range_spec["max"]))
        ]
    elif parameter_type == "int":
        descriptor.integer_range = [
            IntegerRange(from_value=int(range_spec["min"]), to_value=int(range_spec["max"]))
        ]
    else:
        raise ValueError(f'"range" solo se admite en parámetros "double"/"int", no "{parameter_type}"')
    return descriptor


def _resolve_topic(spec: dict) -> TopicSpec:
    # "pkg/msg/Tipo" (string) -> la clase Python real -- misma resolución
    # que usa `ros2 topic pub`/`ros2 topic echo` por dentro (rosidl), no
    # reinventada aquí.
    return TopicSpec(
        topic=spec["topic"],
        message_type=get_message(spec["message_type"]),
        qos=_resolve_qos(spec["qos"]),
        callback=spec.get("callback"),
    )


def load_node_config(path: str) -> NodeConfig:
    with open(path) as config_file:
        raw = yaml.safe_load(config_file)

    parameters = [
        ParameterSpec(name=name, default=spec["default"], descriptor=_build_descriptor(spec))
        for name, spec in raw.get("parameters", {}).items()
    ]
    timers = [
        TimerSpec(period_parameter=spec["period_parameter"], callback=spec["callback"])
        for spec in raw.get("timers", [])
    ]

    return NodeConfig(
        node_name=raw["node"]["name"],
        namespace=raw["node"].get("namespace", ""),
        parameters=parameters,
        publishers=[_resolve_topic(spec) for spec in raw.get("publishers", [])],
        subscriptions=[_resolve_topic(spec) for spec in raw.get("subscriptions", [])],
        timers=timers,
    )


def apply_node_config(node: Node, config: NodeConfig) -> Dict[str, Any]:
    """Ejecuta sobre `node` (ya construido -- ver docstring del módulo) las
    llamadas rclpy que antes estaban sueltas y a mano en cada `__init__`.
    Devuelve los publishers creados, indexados por nombre de topic, para que
    el nodo los guarde y los use en su propia lógica -- publicar sigue
    siendo decisión del dominio, no de la config."""
    # 1. Parámetros primero: todo lo demás (el período de un timer, el
    # target de un adaptador) puede depender de su valor ya declarado.
    for parameter in config.parameters:
        node.declare_parameter(parameter.name, parameter.default, parameter.descriptor)

    # 2. Publishers: uno por entrada, en un dict devuelto -- el nodo real
    # los indexa como quiera usarlos (node._publishers["joint_states"]...).
    publishers = {
        topic_spec.topic: node.create_publisher(topic_spec.message_type, topic_spec.topic, topic_spec.qos)
        for topic_spec in config.publishers
    }

    # 3. Subscriptions: el callback se busca POR NOMBRE en el propio nodo --
    # aquí la config dice "dónde está implementado" sin saber cómo; quien de
    # verdad tiene el método (`_on_goal`, etc.) es la clase del nodo.
    for topic_spec in config.subscriptions:
        if topic_spec.callback is None:
            raise ValueError(f'subscription a "{topic_spec.topic}" sin "callback" declarado')
        callback = getattr(node, topic_spec.callback)
        node.create_subscription(topic_spec.message_type, topic_spec.topic, callback, topic_spec.qos)

    # 4. Timers: el período no es un literal del YAML, es el VALOR de un
    # parámetro ya declarado en el paso 1 -- así sigue siendo el mismo
    # número que ve `ros2 param get`, no una copia suelta.
    for timer_spec in config.timers:
        period = float(node.get_parameter(timer_spec.period_parameter).value)
        node.create_timer(period, getattr(node, timer_spec.callback))

    return publishers


def package_config_path(package_name: str, filename: str) -> str:
    """Ruta a un YAML de config instalado en `share/<package_name>/config/`
    -- mismo mecanismo que ya usa cada paquete para `resource/` en su
    `setup.py`, aplicado a `config/`."""
    return os.path.join(get_package_share_directory(package_name), "config", filename)
