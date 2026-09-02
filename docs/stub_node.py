#!/usr/bin/env python3
"""STUB DE REFERENCIA -- NO es un nodo real de este repositorio ni forma
parte de ningún paquete (deliberado: vive en docs/, fuera de src/, para
que quede claro que es material de estudio, no infraestructura). Muestra,
en un solo nodo, TODOS los elementos que puede tener un nodo ROS2 en este
repo, con la MISMA convención que ya usan `robot_node`/`controller_node`/
`commander` (ver `docs/nodos_ros2.md`) -- no es un tutorial genérico de
ROS2, es "así se escribe un nodo aquí".

Bórrame o sustitúyeme en cuanto ya no haga falta: el objetivo es que
sirvas de referencia mientras se construye `perception_node` de verdad,
no quedarte como código permanente.

Elementos que muestra, uno de cada -- el pseudo-perceptor real
probablemente no necesite todos:
  1. Parámetros ROS2 (`declare_parameter`/`get_parameter`).
  2. Wiring de dominio: el `Node` no decide nada por sí mismo, delega en
     un puerto de `shared_kernel` (aquí, uno de mentira) elegido por
     parámetro -- el mismo patrón `_build_adapter` que ya usan
     `RobotNode`/`ControllerNode`. Es la pieza que de verdad distingue un
     nodo "de este repo" de un tutorial genérico.
  3. Subscription, con su callback privado `_on_xxx`.
  4. Publisher, con QoS EXPLÍCITA y documentada (mismo patrón que
     `GOAL_QOS`/`STRATEGY_QOS` en `ros2_kit/qos.py`: nunca uses la QoS por
     defecto sin pensar por qué).
  5. Timer -- para publicar/actuar periódicamente, no solo por evento.
  6. Service (SERVIDOR) -- pregunta/respuesta síncrona. Ningún nodo de
     este repo lo usa hoy (`docs/nodos_ros2.md` §6: todo es topics), pero
     es un mecanismo real de ROS2 y vale la pena verlo antes de decidir
     que no hace falta.
  7. Service (CLIENTE) -- llamando a un servicio de OTRO nodo, de forma
     asíncrona (no bloquea el nodo mientras espera respuesta).
  8. Capacidad cosmética OPCIONAL vía `getattr` -- mismo patrón que
     `RobotNode._on_goal` con `mark_goal`: comprobsatr si el adaptador la
     ofrece, sin exigirla en el puerto.
  9. Logging (`self.get_logger()`).
  10. `main()` + `run_node()` de `ros2_kit`, igual que el resto de nodos.
  11. Action (SERVIDOR) -- pregunta/respuesta de LARGA duración, con
      progreso intermedio (`feedback`) y cancelación, a diferencia de un
      Service (síncrono, sin progreso) o un topic (sin ninguna noción de
      "esto es UNA petición con UN resultado"). Ningún nodo de este repo
      las usa hoy -- lo más parecido es la "acción casera" de `feedback`
      por topic que ya documenta `docs/nodos_ros2.md`. Usa la acción
      `Fibonacci` de `example_interfaces` (viene con ROS2, no hace falta
      definir una interfaz `.action` propia solo para verlo funcionar --
      eso exigiría un paquete `ament_cmake` con `rosidl_generate_interfaces`,
      más peso del que hace falta para un stub de referencia).
  12. Action (CLIENTE) -- enviar un goal, recibir feedback varias veces
      mientras se ejecuta, y por último el resultado -- tres callbacks
      distintos, a diferencia del único callback de un Service cliente.

Cómo probarlo (una terminal para el nodo, otra para trastear):
    # terminal 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # para que ros2_kit sea importable
    python3 docs/stub_node.py

    # terminal 2 (con el mismo source /opt/ros/humble/setup.bash)
    ros2 node info /stub_node
    ros2 topic list
    ros2 topic echo /stub_output
    ros2 topic pub --once /stub_input std_msgs/msg/String "{data: hola}"
    ros2 service call /stub_service example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
    ros2 param get /stub_node publish_period_seconds
    ros2 action list
    ros2 action send_goal /stub_fibonacci example_interfaces/action/Fibonacci "{order: 6}" --feedback
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Permite `python3 docs/stub_node.py` desde la raíz del repo sin instalar
# nada -- añade src/ros2_kit al path para poder reutilizar run_node() igual
# que un nodo real (en un paquete de verdad esto lo resuelve colcon solo).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "ros2_kit"))

from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros2_kit import run_node
from std_msgs.msg import String


# --- 2. Puerto de dominio DE MENTIRA -- en un nodo real esto sería un
# Protocol importado de shared_kernel (RobotControllerPort, PerceptionPort,
# ...), no una clase inventada aquí.
class _StubPort:
    def do_something(self, payload: str) -> str:
        raise NotImplementedError


class _EchoAdapter(_StubPort):
    """El adaptador más simple que satisface _StubPort -- mismo papel que
    NaiveTestKinematicsAdapter/StaticPerceptionAdapter: cablea el puerto
    con algo real, no resuelve ningún problema de verdad."""

    def do_something(self, payload: str) -> str:
        return f"echo: {payload}"

    def extra_debug_hook(self, payload: str) -> None:
        """Capacidad cosmética OPCIONAL -- no forma parte de _StubPort,
        ver StubNode._on_input y el punto 8 del docstring del módulo."""
        print(f"[extra_debug_hook] {payload!r}")


# --- 4. QoS de ejemplo -- documentar el PORQUÉ, no solo los valores
# (mismo patrón que ros2_kit/qos.py). Esta es RELIABLE + VOLATILE: se
# garantiza la entrega a quien ya esté suscrito, pero no se retiene el
# último mensaje para quien se suscriba más tarde (a diferencia de
# GOAL_QOS, que sí lo hace porque ahí sí importa el orden de arranque).
_STUB_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class StubNode(Node):
    def __init__(self) -> None:
        super().__init__("stub_node")

        # 1. Parámetros: se declaran con un valor por defecto y se leen
        # UNA VEZ en __init__. Si hiciera falta reaccionar a que cambien
        # en caliente (sin relanzar el nodo), hace falta un callback de
        # parámetros aparte -- ver set_strategy en controller_node.py para
        # el patrón que este repo usa en su lugar (un topic, no un
        # parámetro dinámico).
        self.declare_parameter("adapter_target", "simulado")
        self.declare_parameter("publish_period_seconds", 1.0)
        target = self.get_parameter("adapter_target").value
        period = float(self.get_parameter("publish_period_seconds").value)

        # 2. Wiring de dominio -- ver _build_adapter más abajo.
        self._port = self._build_adapter(target)

        # 3. Subscription + callback privado.
        self._input_sub = self.create_subscription(
            String, "stub_input", self._on_input, 10
        )

        # 4. Publisher, con la QoS de ejemplo de arriba.
        self._output_pub = self.create_publisher(String, "stub_output", _STUB_QOS)

        # 5. Timer -- llama a _on_timer cada `period` segundos, sin
        # esperar a que llegue nada por topic.
        self._timer = self.create_timer(period, self._on_timer)

        # 6. Service SERVIDOR -- responde a quien le llame en
        # "stub_service" (ver _on_service_request).
        self._service = self.create_service(
            AddTwoInts, "stub_service", self._on_service_request
        )

        # 7. Service CLIENTE -- para llamar a un servicio de OTRO nodo.
        # Aquí no hay ningún servidor real al otro lado (nadie sirve
        # "otro_servicio") -- ver _call_other_service_example para cómo
        # se usaría de verdad.
        self._other_service_client = self.create_client(AddTwoInts, "otro_servicio")

        # 11. Action SERVIDOR -- a diferencia de create_service (un único
        # callback), una acción necesita al menos execute_callback; los
        # otros dos (goal_callback, cancel_callback) son opcionales -- sin
        # ellos, ROS2 acepta cualquier goal y no permite cancelar.
        self._fibonacci_action_server = ActionServer(
            self,
            Fibonacci,
            "stub_fibonacci",
            execute_callback=self._execute_fibonacci,
            goal_callback=self._on_fibonacci_goal,
            cancel_callback=self._on_fibonacci_cancel,
        )

        # 12. Action CLIENTE -- para pedirle a OTRO nodo que ejecute una
        # acción suya. Aquí no hay ningún servidor real al otro lado (nadie
        # sirve "otra_accion") -- ver _send_fibonacci_goal_example.
        self._other_action_client = ActionClient(self, Fibonacci, "otra_accion")

        # 9. Logging -- mismo patrón que el resto de nodos: un log de
        # "listo" al final de __init__, con los parámetros relevantes.
        self.get_logger().info(f'stub_node listo, adapter_target="{target}"')

    def _build_adapter(self, target: str) -> _StubPort:
        # Mismo patrón que RobotNode._build_adapter/ControllerNode._build_adapter:
        # el valor del parámetro decide QUÉ adaptador concreto se
        # construye, sin que el resto del nodo sepa cuál es.
        if target == "simulado":
            return _EchoAdapter()
        raise ValueError(f'adapter_target desconocido: "{target}"')

    def _on_input(self, msg: String) -> None:
        # 8. Capacidad cosmética opcional -- getattr en vez de exigirla en
        # el puerto (mismo patrón que RobotNode._on_goal con mark_goal).
        maybe_extra = getattr(self._port, "extra_debug_hook", None)
        if maybe_extra is not None:
            maybe_extra(msg.data)

        result = self._port.do_something(msg.data)
        self._output_pub.publish(String(data=result))

    def _on_timer(self) -> None:
        self._output_pub.publish(String(data="tick"))

    def _on_service_request(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        # Los servicios son síncronos desde el punto de vista de quien
        # llama: se bloquea hasta que este callback devuelve `response`.
        response.sum = request.a + request.b
        return response

    def _call_other_service_example(self) -> None:
        # Ejemplo de cómo se LLAMARÍA a un servicio de otro nodo -- nadie
        # invoca esto automáticamente en el stub (no hay servidor real al
        # otro lado). call_async no bloquea: hay que registrar un callback
        # para cuando llegue la respuesta, no se puede simplemente esperar
        # a que vuelva como una llamada de función normal.
        if not self._other_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("otro_servicio no disponible")
            return
        request = AddTwoInts.Request(a=1, b=2)
        future = self._other_service_client.call_async(request)
        future.add_done_callback(self._on_other_service_response)

    def _on_other_service_response(self, future) -> None:
        self.get_logger().info(f"respuesta de otro_servicio: {future.result()}")

    def _on_fibonacci_goal(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        # goal_callback: aceptar o rechazar el goal ANTES de empezar a
        # ejecutarlo -- aquí, un rechazo simple de validación (un Service
        # no tiene este paso: o lo atiendes, o no te suscribes).
        if goal_request.order < 1:
            self.get_logger().warning("Fibonacci: order debe ser >= 1, rechazado")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_fibonacci_cancel(self, goal_handle) -> CancelResponse:
        # cancel_callback: se puede rechazar una cancelación (p. ej. si ya
        # no es seguro parar a mitad) -- aquí siempre se acepta.
        self.get_logger().info("Fibonacci: cancelación solicitada")
        return CancelResponse.ACCEPT

    def _execute_fibonacci(self, goal_handle) -> Fibonacci.Result:
        # execute_callback: el cuerpo real de la acción -- se ejecuta
        # DESPUÉS de que goal_callback haya aceptado. A diferencia de un
        # Service (que responde de una vez), aquí se puede ir publicando
        # progreso (`publish_feedback`) mientras se trabaja, y comprobar
        # si han pedido cancelar a mitad de camino.
        sequence = [0, 1]
        for _ in range(goal_handle.request.order - 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result(sequence=sequence)
            sequence.append(sequence[-1] + sequence[-2])
            feedback = Fibonacci.Feedback(sequence=sequence)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)  # simula trabajo real que tarda -- no bloquea a nadie más

        goal_handle.succeed()
        return Fibonacci.Result(sequence=sequence)

    def _send_fibonacci_goal_example(self) -> None:
        # Ejemplo de cómo se ENVIARÍA un goal a la acción de otro nodo --
        # nadie invoca esto automáticamente en el stub. send_goal_async
        # (como call_async en los servicios) no bloquea -- hace falta
        # encadenar callbacks: primero si el goal fue aceptado, luego el
        # feedback (varias veces), y por último el resultado.
        if not self._other_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("otra_accion no disponible")
            return
        goal = Fibonacci.Goal(order=6)
        send_goal_future = self._other_action_client.send_goal_async(
            goal, feedback_callback=self._on_fibonacci_feedback
        )
        send_goal_future.add_done_callback(self._on_fibonacci_goal_response)

    def _on_fibonacci_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("otra_accion rechazó el goal")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_fibonacci_result)

    def _on_fibonacci_feedback(self, feedback_msg) -> None:
        self.get_logger().info(f"progreso: {feedback_msg.feedback.sequence}")

    def _on_fibonacci_result(self, future) -> None:
        self.get_logger().info(f"resultado final: {future.result().result.sequence}")


# 10. main() + run_node() -- idéntico al de cualquier nodo real del repo.
def main(args=None):
    run_node(StubNode, args=args)


if __name__ == "__main__":
    main()
