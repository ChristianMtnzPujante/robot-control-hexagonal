"""Ciclo de vida estándar de un nodo rclpy, encapsulado una sola vez.

Antes, robot_node y controller_node repetían el mismo
init -> crear -> spin -> destroy -> shutdown en su main().
"""

from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.node import Node


def run_node(node_factory: Callable[[], Node], args=None) -> None:
    """Para el caso simple: crear el nodo y quedarse escuchando de inmediato."""
    rclpy.init(args=args)
    node = node_factory()
    try:
        rclpy.spin(node)
    finally:
        shutdown_node(node)


def shutdown_node(node: Node) -> None:
    """Solo el apagado -- para casos como el Commander, que necesita hacer
    algo (crear sesiones, mandar un goal) entre init/crear y spin, y por
    tanto no puede usar run_node() tal cual.
    """
    node.destroy_node()
    rclpy.shutdown()
