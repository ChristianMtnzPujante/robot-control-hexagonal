"""Cr5RealRobotAdapter contra dos servidores TCP de mentira (comandos y
real-time) en puertos efímeros -- valida el cableado del adaptador
(RobotConnectorPort <-> protocolo), no el robot físico (ver el docstring de
_cr5_protocol.py: eso sigue sin verificar, no disponible en este entorno).

Un único servidor de comandos, no dos, porque el manual oficial del
fabricante (ver _cr5_protocol.py) confirma que RequestControl(),
EnableRobot() y MovJ() comparten el mismo puerto 29999 -- no hay un puerto
de movimiento separado, a diferencia de lo que asumía la primera versión
de este archivo. RequestControl() se añadió tras la primera prueba real
contra el robot físico (ver Cr5RealRobotAdapter._ensure_enabled).
"""

from __future__ import annotations

import math
import socket
import struct
import threading
from typing import Callable, List, Tuple

import pytest

from shared_kernel import JointConfiguration, JointPosition

from robot_node.adapters._cr5_protocol import Cr5ProtocolError
from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_TEST_VALUE_OFFSET = 48
_EXPECTED_TEST_VALUE = 0x0123456789ABCDEF
_Q_ACTUAL_OFFSET = 432
_FRAME_LENGTH = 1440


def _serve_forever(on_connection: Callable[[socket.socket], None]) -> Tuple[int, threading.Thread]:
    """Como el `_serve_once` de test_cr5_protocol.py, pero acepta conexiones
    en bucle en vez de una sola vez: get_robot_mode() cierra su propia
    conexión cuando la abre ella misma (ver su docstring en
    cr5_real_adapter.py), así que un mismo Cr5RealRobotAdapter puede abrir
    varias conexiones TCP sucesivas a lo largo de un test, no solo una."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(5)
    port = server.getsockname()[1]

    def _run() -> None:
        try:
            while True:
                conn, _ = server.accept()
                try:
                    on_connection(conn)
                except OSError:
                    pass  # el test ya cerró todo lo que necesitaba cerrar
                finally:
                    conn.close()
        except OSError:
            pass  # server.close() desde el test (fin del test) rompe accept()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return port, thread


def _command_recording(
    received: List[bytes],
    movj_response: bytes = b"0,{1},MovJ();",
    robot_mode_response: bytes = b"0,{4},RobotMode();",
    disable_response: bytes = b"0,{},DisableRobot();",
) -> Callable[[socket.socket], None]:
    """Responde RequestControl()/EnableRobot() con éxito siempre, y
    RobotMode()/MovJ()/DisableRobot() con sus respectivos `*_response`
    (todos con éxito por defecto) -- cada uno por separado, no un mismo
    "else" para MovJ y DisableRobot: en la realidad un MovJ rechazado no
    implica que DisableRobot también lo esté, y adapter.close() (que
    intenta disable() si el robot sigue enabled) necesita poder distinguir
    los dos en los tests que fuerzan un error solo en MovJ."""

    def _serve(conn: socket.socket) -> None:
        while True:
            command = conn.recv(1024)
            if not command:
                return
            received.append(command)
            if command.startswith(b"RequestControl"):
                conn.sendall(b"0,{},RequestControl();")
            elif command.startswith(b"EnableRobot"):
                conn.sendall(b"0,{},EnableRobot();")
            elif command.startswith(b"RobotMode"):
                conn.sendall(robot_mode_response)
            elif command.startswith(b"DisableRobot"):
                conn.sendall(disable_response)
            else:
                conn.sendall(movj_response)

    return _serve


def _realtime_streaming(angles_deg: List[float]) -> Callable[[socket.socket], None]:
    frame = bytearray(_FRAME_LENGTH)
    struct.pack_into("<Q", frame, _TEST_VALUE_OFFSET, _EXPECTED_TEST_VALUE)
    struct.pack_into("<6d", frame, _Q_ACTUAL_OFFSET, *angles_deg)

    def _serve(conn: socket.socket) -> None:
        try:
            while True:
                conn.sendall(bytes(frame))
        except OSError:
            return

    return _serve


@pytest.fixture()
def fake_cr5():
    """Levanta los dos servidores de mentira y construye un
    Cr5RealRobotAdapter apuntando a sus puertos efímeros -- cada test
    decide qué manda el de comandos/real-time falso vía los parámetros."""

    def _make(command_handler, realtime_angles_deg, **adapter_kwargs):
        commands_received: List[bytes] = []
        command_port, command_thread = _serve_forever(command_handler(commands_received))
        realtime_port, realtime_thread = _serve_forever(
            _realtime_streaming(realtime_angles_deg)
        )
        adapter = Cr5RealRobotAdapter(
            "127.0.0.1",
            joint_names=_JOINT_NAMES,
            command_port=command_port,
            realtime_port=realtime_port,
            **adapter_kwargs,
        )
        return adapter, commands_received

    yield _make


def test_constructor_does_not_enable_the_robot(fake_cr5):
    # RequestControl()/EnableRobot() se posponen al primer set_joints --
    # construir el adaptador no debe, por sí solo, energizar los servos ni
    # dejar el socket de comandos abierto e inactivo (ver docstring de
    # Cr5RealRobotAdapter.__init__ sobre por qué esto último importa).
    adapter, _ = fake_cr5(_command_recording, [0.0] * 6)
    try:
        assert adapter._enabled is False
        assert adapter._commands._sock is None
    finally:
        adapter.close()


def test_set_joints_enables_once_and_sends_movj_with_joint_braces_in_degrees(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        positions = [
            JointPosition(name, angle_radians=math.radians(value))
            for name, value in zip(_JOINT_NAMES, [10.0, -20.0, 30.0, 0.0, 90.0, -45.0])
        ]
        configuration = JointConfiguration.create(positions).value

        adapter.set_joints(configuration)

        assert adapter._enabled is True
        # RequestControl() + EnableRobot() + un único MovJ, en ese orden.
        assert [c.split(b"(")[0] for c in commands_received] == [
            b"RequestControl",
            b"EnableRobot",
            b"MovJ",
        ]
        movj_command = commands_received[2].decode("ascii")
        assert movj_command.startswith("MovJ(joint={")
        assert movj_command.endswith(",cp=50)")  # _DEFAULT_MOVJ_CP
        assert "10.000" in movj_command and "-45.000" in movj_command

        # Un segundo set_joints no debe repetir RequestControl()/EnableRobot().
        adapter.set_joints(configuration)
        assert len(commands_received) == 4
        assert commands_received[3].startswith(b"MovJ(")
    finally:
        adapter.close()


def test_set_joints_honours_a_custom_movj_cp(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6, movj_cp=100)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)
        movj_command = commands_received[-1].decode("ascii")
        assert movj_command.endswith(",cp=100)")
    finally:
        adapter.close()


def test_set_joints_raises_on_a_non_zero_error_code(fake_cr5):
    def _command_rejecting_movj(received):
        return _command_recording(received, movj_response=b"-1,{},MovJ();")

    adapter, _ = fake_cr5(_command_rejecting_movj, [0.0] * 6)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        with pytest.raises(Cr5ProtocolError):
            adapter.set_joints(configuration)
    finally:
        adapter.close()


def test_set_joints_raises_if_request_control_is_rejected(fake_cr5):
    def _command_rejecting_request_control(received):
        def _serve(conn: socket.socket) -> None:
            while True:
                command = conn.recv(1024)
                if not command:
                    return
                received.append(command)
                if command.startswith(b"RequestControl"):
                    conn.sendall(b"-1,{},RequestControl();")
                else:
                    conn.sendall(b"0,{},EnableRobot();")

        return _serve

    adapter, commands_received = fake_cr5(_command_rejecting_request_control, [0.0] * 6)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        with pytest.raises(Cr5ProtocolError):
            adapter.set_joints(configuration)
        # No debe haber intentado EnableRobot() tras el rechazo.
        assert commands_received == [b"RequestControl()"]
    finally:
        adapter.close()


def test_set_joints_rejects_an_angle_beyond_the_joint_limit_without_sending_anything(fake_cr5):
    # joint3 (índice 2) es el más restrictivo del CR5: ±160° (ver
    # _JOINT_LIMITS_DEGREES) -- 161° lo excede.
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        positions = [
            JointPosition(name, math.radians(161.0 if name == "joint3" else 0.0))
            for name in _JOINT_NAMES
        ]
        configuration = JointConfiguration.create(positions).value
        with pytest.raises(Cr5ProtocolError, match="joint3"):
            adapter.set_joints(configuration)
        # Rechazado ANTES de mandar nada -- ni EnableRobot() siquiera.
        assert commands_received == []
        assert adapter.is_enabled is False
    finally:
        adapter.close()


def test_set_joints_allows_an_angle_exactly_at_the_joint_limit(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        positions = [
            JointPosition(name, math.radians(160.0 if name == "joint3" else 0.0))
            for name in _JOINT_NAMES
        ]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)  # no debe lanzar
        assert any(c.startswith(b"MovJ(") for c in commands_received)
    finally:
        adapter.close()


def test_set_joints_rejects_a_wide_range_joint_beyond_360_degrees(fake_cr5):
    # Defensa en profundidad para el bug real ya visto en este proyecto:
    # una IK convergiendo a una vuelta de más (ver nota "CORREGIDO 07/09
    # (Bloque 0 #114)" en cr5_real_adapter.py) -- joint1 permite ±360°,
    # 361° lo excede.
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        positions = [
            JointPosition(name, math.radians(-361.0 if name == "joint1" else 0.0))
            for name in _JOINT_NAMES
        ]
        configuration = JointConfiguration.create(positions).value
        with pytest.raises(Cr5ProtocolError, match="joint1"):
            adapter.set_joints(configuration)
        assert commands_received == []
    finally:
        adapter.close()


def test_get_current_configuration_converts_degrees_to_radians(fake_cr5):
    angles_deg = [10.0, -20.0, 30.0, 0.0, 90.0, -45.0]
    adapter, _ = fake_cr5(_command_recording, angles_deg)
    try:
        configuration = adapter.get_current_configuration()
        for name, expected_deg in zip(_JOINT_NAMES, angles_deg):
            assert configuration.angle_of(name) == pytest.approx(math.radians(expected_deg))
    finally:
        adapter.close()


def test_get_robot_mode_parses_the_value_between_braces(fake_cr5):
    adapter, _ = fake_cr5(_command_recording, [0.0] * 6)
    try:
        mode, description = adapter.get_robot_mode()
        assert mode == 4
        assert "des-energizado" in description
    finally:
        adapter.close()


def test_get_robot_mode_reports_an_uncontrollable_mode(fake_cr5):
    def _command_reporting_mode_5(received):
        return _command_recording(received, robot_mode_response=b"0,{5},RobotMode();")

    adapter, _ = fake_cr5(_command_reporting_mode_5, [0.0] * 6)
    try:
        mode, description = adapter.get_robot_mode()
        assert mode == 5
        assert "habilitado" in description
    finally:
        adapter.close()


def test_get_robot_mode_closes_its_own_connection_so_set_joints_starts_fresh(fake_cr5):
    # Regresión (segunda prueba real): get_robot_mode() (paso 1 de
    # cr5_first_contact_demo) abre el socket de comandos solo para esa
    # consulta y lo vuelve a cerrar -- si se dejara abierto, el tiempo que
    # tarda una persona en leer y confirmar por teclado antes de llegar al
    # movimiento deja esa conexión inactiva el tiempo suficiente para que
    # el CR5 real la mate (visto como "timed out" en RequestControl()).
    # set_joints() debe poder abrir su propia conexión nueva sin problema.
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        adapter.get_robot_mode()
        assert adapter._commands.is_connected is False

        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)
        assert [c.split(b"(")[0] for c in commands_received] == [
            b"RobotMode",
            b"RequestControl",
            b"EnableRobot",
            b"MovJ",
        ]
    finally:
        adapter.close()


def test_disable_sends_disablerobot_and_clears_enabled_flag(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)

        adapter.disable()

        assert adapter.is_enabled is False
        assert commands_received[-1] == b"DisableRobot()"
    finally:
        adapter.close()


def test_disable_reconnects_without_request_control_if_the_connection_died(fake_cr5):
    # Regresión (robot físico, cr5_repeated_joint1_moves_demo.py): el
    # socket de comandos puede morir por inactividad entre un movimiento y
    # el siguiente. disable() debe reconectar y, CONFIRMADO en la práctica
    # con cr5_disable_demo.py, mandar DisableRobot() directamente en la
    # conexión nueva -- sin RequestControl() previo, porque el modo TCP es
    # un estado del robot, no de la conexión que lo pidió.
    state = {"connections": 0}

    def _command_dying_before_disable(received):
        def _serve(conn: socket.socket) -> None:
            state["connections"] += 1
            first_connection = state["connections"] == 1
            while True:
                command = conn.recv(1024)
                if not command:
                    return
                received.append(command)
                if command.startswith(b"RequestControl"):
                    conn.sendall(b"0,{},RequestControl();")
                elif command.startswith(b"EnableRobot"):
                    conn.sendall(b"0,{},EnableRobot();")
                elif command.startswith(b"DisableRobot"):
                    if first_connection:
                        return  # simula la conexión ya muerta: cierra sin responder
                    conn.sendall(b"0,{},DisableRobot();")
                else:
                    conn.sendall(b"0,{},MovJ();")

        return _serve

    adapter, commands_received = fake_cr5(_command_dying_before_disable, [0.0] * 6)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)  # deja _commands conectada y enabled

        adapter.disable()

        assert adapter.is_enabled is False
        # Segunda conexión: solo DisableRobot(), sin repetir RequestControl().
        assert commands_received[-2:] == [b"DisableRobot()", b"DisableRobot()"]
    finally:
        adapter.close()


def test_disable_raises_on_a_non_zero_error_code(fake_cr5):
    def _command_rejecting_disable(received):
        def _serve(conn: socket.socket) -> None:
            while True:
                command = conn.recv(1024)
                if not command:
                    return
                received.append(command)
                if command.startswith(b"DisableRobot"):
                    conn.sendall(b"-1,{},DisableRobot();")
                elif command.startswith(b"RequestControl"):
                    conn.sendall(b"0,{},RequestControl();")
                else:
                    conn.sendall(b"0,{},EnableRobot();")

        return _serve

    adapter, _ = fake_cr5(_command_rejecting_disable, [0.0] * 6)
    try:
        positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
        configuration = JointConfiguration.create(positions).value
        adapter.set_joints(configuration)
        with pytest.raises(Cr5ProtocolError):
            adapter.disable()
    finally:
        # El mismo servidor de mentira sigue rechazando DisableRobot() --
        # close() (que reintenta disable() porque is_enabled sigue True,
        # el disable() de arriba falló antes de limpiarlo) cierra los
        # sockets igualmente pero vuelve a relanzar el mismo error al
        # final (comportamiento a propósito, ver su docstring) -- esperado
        # aquí, no un fallo del test.
        with pytest.raises(Cr5ProtocolError):
            adapter.close()


def test_close_disables_the_robot_if_it_was_still_enabled(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
    configuration = JointConfiguration.create(positions).value
    adapter.set_joints(configuration)  # deja is_enabled en True

    adapter.close()

    assert adapter.is_enabled is False
    assert commands_received[-1] == b"DisableRobot()"
    assert adapter._commands.is_connected is False
    assert adapter._realtime._sock is None


def test_close_still_closes_both_sockets_even_if_disable_fails(fake_cr5):
    def _command_rejecting_disable(received):
        def _serve(conn: socket.socket) -> None:
            while True:
                command = conn.recv(1024)
                if not command:
                    return
                received.append(command)
                if command.startswith(b"DisableRobot"):
                    conn.sendall(b"-1,{},DisableRobot();")
                elif command.startswith(b"RequestControl"):
                    conn.sendall(b"0,{},RequestControl();")
                else:
                    conn.sendall(b"0,{},EnableRobot();")

        return _serve

    adapter, _ = fake_cr5(_command_rejecting_disable, [0.0] * 6)
    positions = [JointPosition(name, 0.0) for name in _JOINT_NAMES]
    configuration = JointConfiguration.create(positions).value
    adapter.set_joints(configuration)

    with pytest.raises(Cr5ProtocolError):
        adapter.close()

    # El fallo de disable() no debe impedir que los sockets se cierren --
    # "mejor esfuerzo", no "todo o nada" (ver docstring de close()).
    assert adapter._commands.is_connected is False
    assert adapter._realtime._sock is None


def test_close_does_not_attempt_to_disable_if_never_enabled(fake_cr5):
    adapter, commands_received = fake_cr5(_command_recording, [0.0] * 6)
    adapter.close()
    assert commands_received == []


def test_constructor_rejects_a_joint_names_list_of_the_wrong_length():
    with pytest.raises(ValueError):
        Cr5RealRobotAdapter("127.0.0.1", joint_names=["only_one_joint"])
