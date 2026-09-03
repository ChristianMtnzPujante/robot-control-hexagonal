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

# STRATEGY_QOS es la contraria a propósito: perder un cambio de estrategia
# no es crítico -- la sesión sigue funcionando con la estrategia anterior
# (que sigue siendo válida), y quien lo mandó puede simplemente reenviarlo
# si no ve el "estrategia_cambiada" en feedback. No pagamos el coste de
# acks/reintentos de RELIABLE para un canal de control best-effort.
STRATEGY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# SCENE_QOS: mismo razonamiento que GOAL_QOS, mismo problema de arranque.
# Un perceptor (perception_node) y quien lo escuche (Commander) son
# procesos separados que no se coordinan para arrancar en orden -- sin
# TRANSIENT_LOCAL, si Commander se suscribe después de que ya se haya
# publicado la primera Scene, se queda sin saber nada hasta el siguiente
# ciclo del timer del perceptor (o para siempre, si el perceptor solo
# publica una vez). Retener la última Scene publicada elimina esa espera.
SCENE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
