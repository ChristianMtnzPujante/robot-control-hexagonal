"""Adaptador de salida (driven adapter) hacia el CR5 físico.

Implementa RobotConnectorPort hablando directamente el protocolo TCP/IP
propio de Dobot (ver _cr5_protocol.py), sin pasar por el driver oficial
(que es ROS1) ni por un puente ros1_bridge -- la vía "reimplementar TCP/IP"
de la decisión de Vikunja Bloque 0 #22 (ver también ROADMAP.md, Bloque 0).

CORREGIDO 04/09: la primera versión de este archivo mandaba
`JointMovJ(j1,...,j6)` por un supuesto puerto de movimiento (30003). Ese
puerto y ese comando salían de leer un driver de referencia de 2021, no
del manual oficial del fabricante -- que sí encontré después y que dice
otra cosa: solo existen 29999/30004/30005/30006, y el movimiento articular
se manda como `MovJ(joint={j1,...,j6})` por el MISMO puerto 29999 (ver
_cr5_protocol.py para el detalle y la fuente exacta). Un único socket de
comandos basta ahora para todo lo que este adaptador necesita.

CORREGIDO 04/09 (2), tras la primera prueba real: `EnableRobot()` devolvió
"Connection reset by peer" contra el robot físico. El manual documenta un
paso previo que ni el driver de 2021 ni mi primera lectura del manual
tenían en cuenta: `RequestControl()` -- "solo en modo TCP se pueden
ejecutar otros comandos TCP" (sección 2.1) -- hay que pedirlo ANTES de
EnableRobot() o de cualquier otro comando. El propio manual dice además
que RequestControl() solo se admite si el robot está sin energizar o
des-energizado (no si ya está "habilitado e inactivo" vía el teach
pendant) -- si vuelve a fallar, lo primero a comprobar es que el robot
esté des-energizado en el panel antes de lanzar esto. La conexión del
socket de comandos también se retrasó (antes se abría en el constructor):
si se deja abierta e inactiva mientras un humano confirma por teclado,
eso también podría explicar por sí solo un reset por inactividad --
ahora se abre justo antes de mandar el primer comando.

SIN VERIFICAR TODAVÍA CONTRA EL ROBOT FÍSICO -- no disponible en este
entorno. _cr5_protocol.py está cubierto por tests contra un servidor TCP de
mentira que imita el protocolo del manual, pero eso valida el protocolo,
no el robot real -- y el propio manual puede no coincidir con la versión
de firmware exacta de este CR5 concreto (confirmarlo es parte del
checklist físico, Bloque 0 #112).

CORREGIDO 07/09 (Bloque 0 #114): set_joints() ahora rechaza (sin mandar
nada) cualquier ángulo fuera del límite mecánico real del joint -- ver
_JOINT_LIMITS_DEGREES. Es defensa en profundidad, no solo teórica: este
mismo proyecto ya tuvo un bug real (IK convergiendo a una vuelta de más,
joint4=-457°/joint6=540°, ver resumen de sesión 01/09) que una validación
así habría atrapado igual de bien que la que ya existe en el
planificador (_within_a_full_turn en los adaptadores de evitación de
obstáculos) -- capas distintas, mismo tipo de error.

Los propios límites: descarté en su día (07/09, antes de esto) los
límites de la URDF local (~360°/~360°/160°/~360°/~360°/~360°) como
"valores de relleno sin verificar" -- error mío. Verificado ahora contra
TRES fuentes independientes (manual de usuario oficial, manual de
hardware oficial, página de producto oficial dobot-robots.com/products/
cr-series/cr5.html): las tres coinciden con la URDF exactamente --
±360° para J1/J2/J4/J5/J6, ±160° para J3 (el codo, el único realmente
restrictivo de los seis). No son de relleno, son los límites mecánicos
reales.

Límites conocidos, deliberadamente NO resueltos aquí (ver Bloque 0 #114):
no valida el SALTO/velocidad entre una configuración y la siguiente
(necesitaría conocer el dt real entre waypoints, más complejo -- queda
para una iteración posterior), y no intenta recuperarse de un estado de
alarma previo (no llama ClearError() -- si el robot ya está en alarma,
EnableRobot() fallará y set_joints() lo reportará como Cr5ProtocolError,
no lo arreglará solo).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from shared_kernel import JointConfiguration, JointPosition

from ._cr5_protocol import (
    CONTROLLABLE_ROBOT_MODES,
    DASHBOARD_PORT,
    REALTIME_PORT,
    ROBOT_MODE_DESCRIPTIONS,
    Cr5CommandSocket,
    Cr5ProtocolError,
    Cr5RealtimeSocket,
)

# Límites articulares mecánicos REALES del CR5 (grados, simétricos --
# ±valor), indexados por posición (0=J1 ... 5=J6) -- ver la nota
# "CORREGIDO 07/09 (Bloque 0 #114)" en el docstring del módulo sobre las
# tres fuentes independientes que los confirman. Vive aquí, no en
# _cr5_protocol.py: son una propiedad física del brazo, no del protocolo
# TCP/IP en sí (a diferencia de los puertos/formato de mensaje).
_JOINT_LIMITS_DEGREES = [360.0, 360.0, 160.0, 360.0, 360.0, 360.0]

# CP ("continuous path", MovJ(...,cp=valor), rango 0-100, manual sección
# "平滑过渡参数") -- ratio de suavizado entre el MovJ actual y el
# siguiente de la cola. Sin especificarlo, el robot usa 0 (parada
# completa en cada punto) -- confirmado en vivo el 07/09: una trayectoria
# de 21 waypoints sin cp se sentía vibrante y lenta, 21 ciclos de
# arranque/parada para un recorrido de pocos grados. 50 es un valor
# intermedio deliberado, no el máximo (100): con cp>0 el robot NO pasa
# exactamente por los puntos intermedios (el manual lo advierte
# explícitamente), así que un valor moderado da suavidad real sin alejarse
# demasiado de la trayectoria calculada por PoE. No aplica al ÚLTIMO
# waypoint de una trayectoria de forma distinta -- si no hay comando
# siguiente en cola, no hay nada que suavizar, así que el punto final
# sigue siendo preciso.
_DEFAULT_MOVJ_CP = 50


class Cr5RealRobotAdapter:
    def __init__(
        self,
        host: str,
        joint_names: List[str],
        command_port: int = DASHBOARD_PORT,
        realtime_port: int = REALTIME_PORT,
        movj_cp: int = _DEFAULT_MOVJ_CP,
    ):
        # joint_names[i] es el nombre de dominio del eje físico J{i+1} del
        # CR5 -- el propio robot no tiene nombres, solo un array ordenado
        # de 6 ángulos (ver q_actual en _cr5_protocol.py), así que esta
        # lista es la única forma de saber a qué joint del dominio
        # corresponde cada posición del array.
        #
        # command_port/realtime_port solo existen para poder apuntar los
        # tests a un servidor de mentira en un puerto efímero, sin tocar
        # 29999/30004 -- en producción siempre son los del fabricante (ver
        # _cr5_protocol.py), nunca configuración de robot_node.yaml.
        if len(joint_names) != 6:
            raise ValueError(
                "Cr5RealRobotAdapter: el CR5 tiene 6 articulaciones, se "
                f"recibieron {len(joint_names)} nombres: {joint_names}"
            )
        self._joint_names = list(joint_names)
        self._movj_cp = movj_cp
        self._commands = Cr5CommandSocket(host, command_port)
        self._realtime = Cr5RealtimeSocket(host, realtime_port)
        self._realtime.connect()
        # El socket de comandos NO se conecta aquí (a diferencia del de
        # real-time) -- se deja para _ensure_enabled, justo antes de
        # mandar el primer comando, para no tenerlo abierto e inactivo
        # mientras tanto (ver nota "CORREGIDO 04/09 (2)" arriba sobre el
        # reset de conexión). EnableRobot() en sí también se pospone al
        # primer set_joints: construir el adaptador (p. ej. al arrancar
        # robot_node) no debería, por sí solo, energizar los servos del
        # robot físico.
        self._enabled = False

    def get_robot_mode(self) -> Tuple[int, str]:
        """Consulta RobotMode() y devuelve (código, descripción legible) --
        diagnóstico, no forma parte de RobotConnectorPort (eso son solo
        get_current_configuration/set_joints). RobotMode() está en la
        sección de consulta del manual, no en la de control, así que puede
        llamarse ANTES de _ensure_enabled/RequestControl() -- es justo lo
        que permite decidir de antemano si RequestControl() va a
        funcionar: solo lo admite si el código está en
        CONTROLLABLE_ROBOT_MODES (ver _cr5_protocol.py).

        Si el socket de comandos no estaba ya conectado (caso típico:
        nadie ha llamado a _ensure_enabled todavía), se cierra otra vez al
        terminar en vez de dejarlo abierto -- descubierto en la segunda
        prueba real: un socket de comandos abierto pero inactivo durante
        el tiempo que tarda una persona en leer y confirmar por teclado
        acaba muerto (visto como "timed out" al mandar RequestControl()
        más tarde), así que _ensure_enabled siempre debe partir de una
        conexión fresca, no de esta. Si ya estaba conectado desde antes
        (p. ej. una sesión ya en curso), se deja tal cual -- no es de esta
        llamada de quien es la conexión. (query() ya reconecta sola si la
        conexión resulta estar muerta -- ver _cr5_protocol.py -- esto solo
        decide si HAY que cerrarla otra vez al terminar, no si hay que
        reintentar.)"""
        opened_here = not self._commands.is_connected
        try:
            error_code, value = self._commands.query("RobotMode()")
        finally:
            if opened_here:
                self._commands.close()
        if error_code != 0:
            raise Cr5ProtocolError(f"RobotMode() devolvió el código de error {error_code}")
        try:
            mode = int(value)
        except ValueError as error:
            raise Cr5ProtocolError(
                f'RobotMode() devolvió un valor no numérico: "{value}"'
            ) from error
        description = ROBOT_MODE_DESCRIPTIONS.get(mode, f"código desconocido ({mode})")
        return mode, description

    def _ensure_enabled(self) -> None:
        if self._enabled:
            return
        # No hace falta conectar a mano aquí: send_command()/query() ya
        # conectan solas si hace falta (ver _cr5_protocol.py) -- incluida
        # la reconexión si get_robot_mode() dejó algo raro a medias.
        #
        # RequestControl() antes que nada más: el manual dice que ningún
        # otro comando TCP (ni EnableRobot) se acepta hasta pedir el modo
        # TCP explícitamente -- paso que la primera versión de este
        # adaptador se saltó por completo (ver nota arriba).
        error_code = self._commands.send_command("RequestControl()")
        if error_code != 0:
            raise Cr5ProtocolError(
                f"RequestControl() devolvió el código de error {error_code} -- "
                "el manual solo permite pedir el modo TCP si el robot está "
                "sin energizar o des-energizado. Si ya está habilitado desde "
                "el teach pendant, des-energízalo ahí primero."
            )
        error_code = self._commands.send_command("EnableRobot()")
        if error_code != 0:
            raise Cr5ProtocolError(
                f"EnableRobot() devolvió el código de error {error_code}"
            )
        self._enabled = True

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def disable(self) -> None:
        """Des-energiza el robot (DisableRobot()) -- pensado como paso
        final tras una prueba real, para no dejar los servos activos sin
        nadie vigilando. Solo tiene sentido llamarlo si is_enabled es True
        (si nunca se llegó a habilitar, no hay nada que deshacer).

        CONFIRMADO EN LA PRÁCTICA (cr5_disable_demo.py, primera vez que se
        pudo probar): DisableRobot() NO exige pedir RequestControl() de
        nuevo en una conexión nueva -- el modo TCP es un estado del propio
        robot, concedido la primera vez que algo lo pidió con éxito, no de
        la conexión concreta que lo pidió. Por eso aquí no se manda
        RequestControl() antes de DisableRobot(), a diferencia de
        _ensure_enabled.

        La reconexión si la conexión existente resulta estar muerta (visto
        en vivo más de una vez: idle de varios segundos, reset a mitad de
        una ráfaga de comandos) ya la hace send_command()/query() sola --
        ver _cr5_protocol.py -- así que aquí basta con mandar el comando.
        """
        error_code = self._commands.send_command("DisableRobot()")
        if error_code != 0:
            raise Cr5ProtocolError(
                f"DisableRobot() devolvió el código de error {error_code}"
            )
        self._enabled = False

    def set_joints(self, configuration: JointConfiguration) -> None:
        angles_deg = [
            math.degrees(configuration.angle_of(name)) for name in self._joint_names
        ]
        # Validar ANTES de habilitar/mandar nada -- si esto rechaza, el
        # robot no debe ni siquiera energizarse para este intento (ver
        # _validate_joint_limits).
        self._validate_joint_limits(angles_deg)
        self._ensure_enabled()
        # MovJ(P, user, tool, a, v, cp): P es obligatorio, el resto
        # opcionales. user/tool/a/v se omiten (usan el default de fábrica
        # del robot, ver Bloque 0 #114 sobre por qué no se fija aquí una
        # velocidad reducida a mano). P es un único parámetro-string
        # "joint={...}", con los 6 ángulos SEPARADOS POR COMAS dentro de
        # las llaves -- no son 6 parámetros posicionales como en el
        # JointMovJ(j1,...,j6) antiguo.
        #
        # cp SÍ se manda explícitamente (ver _DEFAULT_MOVJ_CP) -- sin él,
        # el robot para completamente en cada waypoint de una trayectoria
        # de varios puntos, lo que en una prueba real (07/09) se sintió
        # como vibración y lentitud para un recorrido de pocos grados.
        joint_values = ",".join(f"{angle:.3f}" for angle in angles_deg)
        command = f"MovJ(joint={{{joint_values}}},cp={self._movj_cp})"
        error_code = self._commands.send_command(command)
        if error_code != 0:
            raise Cr5ProtocolError(f"MovJ devolvió el código de error {error_code}")

    def _validate_joint_limits(self, angles_deg: List[float]) -> None:
        """Rechaza (sin mandar NADA al robot, ni siquiera EnableRobot) si
        algún ángulo pedido excede el límite mecánico real de su joint
        (ver _JOINT_LIMITS_DEGREES) -- Bloque 0 #114. Por índice, no por
        nombre: self._joint_names[i] es, por invariante de este adaptador
        (ver __init__), siempre el eje físico J{i+1}, en ese orden."""
        for index, (name, angle_deg) in enumerate(zip(self._joint_names, angles_deg)):
            limit = _JOINT_LIMITS_DEGREES[index]
            if abs(angle_deg) > limit:
                raise Cr5ProtocolError(
                    f'"{name}" (J{index + 1}) pide {angle_deg:.2f}°, fuera de '
                    f"su límite mecánico ±{limit:.0f}° -- MovJ NO se ha mandado."
                )

    def get_current_configuration(self) -> JointConfiguration:
        angles_deg = self._realtime.read_joint_angles_deg()
        positions = [
            JointPosition(name, math.radians(angle))
            for name, angle in zip(self._joint_names, angles_deg)
        ]
        result = JointConfiguration.create(positions)
        # positions nunca está vacío aquí (self._joint_names tiene siempre
        # 6 elementos, validado en __init__), así que este Either siempre
        # es Right -- mismo patrón que CoppeliaSimRobotAdapter.
        return result.value

    def close(self) -> None:
        """Cierra ambos sockets -- parte de RobotConnectorPort (Vikunja
        #116: robot_node no llamaba a esto en NINGÚN camino de cierre,
        normal o por crash, así que cada sesión real de este bloque dejó
        el robot habilitado hasta desconectarlo a mano con
        cr5_disable_demo.py).

        Si el robot seguía habilitado, intenta des-energizarlo antes de
        cerrar -- best-effort: los sockets se cierran igualmente aunque
        disable() falle (nada que ganar dejándolos abiertos), pero el
        error se re-lanza al final para que quien llame a close() (ver
        RobotNode.destroy_node) se entere y pueda avisar de que el robot
        podría haber quedado energizado sin nadie vigilando."""
        disable_error: Optional[Cr5ProtocolError] = None
        if self._enabled:
            try:
                self.disable()
            except Cr5ProtocolError as error:
                disable_error = error
        self._commands.close()
        self._realtime.close()
        if disable_error is not None:
            raise disable_error
