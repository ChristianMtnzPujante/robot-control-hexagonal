import pytest
from std_msgs.msg import String

from ros2_kit.node_config import GOAL_QOS, apply_node_config, load_node_config

_YAML = """
node:
  name: stub_node

parameters:
  publish_period_seconds:
    type: double
    default: 1.0
    range: {min: 0.1, max: 10.0}
  adapter_target:
    type: string
    default: echo

publishers:
  - topic: stub_output
    message_type: std_msgs/msg/String
    qos: GOAL_QOS

subscriptions:
  - topic: stub_input
    message_type: std_msgs/msg/String
    qos: 10
    callback: _on_input

timers:
  - period_parameter: publish_period_seconds
    callback: _on_timer
"""


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    """Doble de pruebas de un rclpy.node.Node -- mismo patrón que los stubs
    de KinematicsPort en controller_node/test: solo registra qué se llamó,
    sin arrancar rclpy de verdad (el repo no tiene tests que sí lo hagan)."""

    def __init__(self):
        self._declared_parameters = {}
        self.created_publishers = []
        self.created_subscriptions = []
        self.created_timers = []

    def declare_parameter(self, name, default, descriptor=None):
        self._declared_parameters[name] = (default, descriptor)

    def get_parameter(self, name):
        default, _descriptor = self._declared_parameters[name]
        return _FakeParameter(default)

    def create_publisher(self, message_type, topic, qos):
        publisher = object()
        self.created_publishers.append((message_type, topic, qos, publisher))
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        self.created_subscriptions.append((message_type, topic, callback, qos))

    def create_timer(self, period, callback):
        self.created_timers.append((period, callback))

    def _on_input(self, msg):
        pass

    def _on_timer(self):
        pass


def _write_config(tmp_path, text=_YAML):
    path = tmp_path / "stub_node.yaml"
    path.write_text(text)
    return str(path)


def test_load_node_config_resolves_message_types_and_named_qos(tmp_path):
    config = load_node_config(_write_config(tmp_path))

    assert config.node_name == "stub_node"
    assert config.publishers[0].message_type is String
    assert config.publishers[0].qos is GOAL_QOS
    assert config.subscriptions[0].qos == 10  # entero -> profundidad simple, no perfil


def test_load_node_config_builds_a_real_range_descriptor_for_numeric_parameters(tmp_path):
    config = load_node_config(_write_config(tmp_path))

    period_spec = next(p for p in config.parameters if p.name == "publish_period_seconds")
    assert period_spec.default == 1.0
    assert period_spec.descriptor.floating_point_range[0].from_value == 0.1
    assert period_spec.descriptor.floating_point_range[0].to_value == 10.0

    target_spec = next(p for p in config.parameters if p.name == "adapter_target")
    assert target_spec.descriptor.floating_point_range == []
    assert target_spec.descriptor.integer_range == []


def test_load_node_config_rejects_range_on_a_non_numeric_parameter(tmp_path):
    text = _YAML.replace(
        "  adapter_target:\n    type: string\n    default: echo",
        "  adapter_target:\n    type: string\n    default: echo\n    range: {min: 0, max: 1}",
    )

    with pytest.raises(ValueError):
        load_node_config(_write_config(tmp_path, text))


def test_apply_node_config_declares_parameters_creates_channels_and_resolves_callbacks_by_name(tmp_path):
    config = load_node_config(_write_config(tmp_path))
    node = _FakeNode()

    publishers = apply_node_config(node, config)

    assert "publish_period_seconds" in node._declared_parameters
    assert "adapter_target" in node._declared_parameters

    assert "stub_output" in publishers
    (message_type, topic, qos, _publisher) = node.created_publishers[0]
    assert (message_type, topic, qos) == (String, "stub_output", GOAL_QOS)

    (message_type, topic, callback, qos) = node.created_subscriptions[0]
    assert (message_type, topic, qos) == (String, "stub_input", 10)
    assert callback == node._on_input  # resuelto por nombre, no reimplementado

    (period, callback) = node.created_timers[0]
    assert period == 1.0  # el valor YA declarado del parámetro, no un literal aparte
    assert callback == node._on_timer


def test_apply_node_config_raises_when_a_subscription_has_no_callback(tmp_path):
    text = _YAML.replace("    callback: _on_input\n", "")
    config = load_node_config(_write_config(tmp_path, text))

    with pytest.raises(ValueError):
        apply_node_config(_FakeNode(), config)
