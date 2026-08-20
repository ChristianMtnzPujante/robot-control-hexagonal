"""QoS compartida entre nodos.

`GOAL_QOS` existe por la condición de carrera real que teníamos: el
Commander publica el objetivo sin esperar confirmación de que
controller_node ya está suscrito (arrancan como procesos aparte, en
paralelo). Con la QoS por defecto (volatile), si el mensaje se publica
antes de que el suscriptor se haya descubierto, se pierde para siempre --
sin error, sin reintento. TRANSIENT_LOCAL hace que el último mensaje
publicado quede retenido y se entregue también a quien se suscriba
después, así que el orden de arranque deja de importar.
"""

from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

GOAL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
