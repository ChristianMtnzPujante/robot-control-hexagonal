"""Valida que hablamos el protocolo que documenta el fabricante -- NO que
el CR5 real responda así (no disponible en este entorno). El servidor de
mentira de estos tests imita exactamente el formato de commander.h
(dobot_bringup, ~/ros2_ws/src/TCP-IP-ROS-6AXis): comandos ASCII
terminados en ';' para dashboard/motion, y tramas binarias de 1440 bytes
con un valor mágico en el byte 48 para el socket real-time.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Callable, Tuple

import pytest

from robot_node.adapters._cr5_protocol import (
    Cr5CommandSocket,
    Cr5ProtocolError,
    Cr5RealtimeSocket,
    _extract_last_frame,
    _parse_error_code,
)

_TEST_VALUE_OFFSET = 48
_EXPECTED_TEST_VALUE = 0x0123456789ABCDEF
_Q_ACTUAL_OFFSET = 432
_FRAME_LENGTH = 1440


def _serve_once(on_connection: Callable[[socket.socket], None]) -> Tuple[int, threading.Thread]:
    """Levanta un servidor TCP en un puerto efímero que acepta UNA conexión,
    se la pasa a `on_connection` y se cierra -- bind/listen ocurren aquí,
    de forma síncrona, así que el puerto devuelto ya admite conexiones
    antes de que esta función retorne (sin sleeps para evitar la carrera)."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def _run() -> None:
        conn, _ = server.accept()
        try:
            on_connection(conn)
        finally:
            conn.close()
            server.close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return port, thread


def _build_frame(joint_angles_deg) -> bytes:
    frame = bytearray(_FRAME_LENGTH)
    struct.pack_into("<Q", frame, _TEST_VALUE_OFFSET, _EXPECTED_TEST_VALUE)
    struct.pack_into("<6d", frame, _Q_ACTUAL_OFFSET, *joint_angles_deg)
    return bytes(frame)


# ---------------------------------------------------------------------
# _parse_error_code / _extract_last_frame -- unidades puras, sin sockets
# ---------------------------------------------------------------------


def test_parse_error_code_reads_the_first_integer_before_the_first_comma():
    assert _parse_error_code(b"0,{},EnableRobot();") == 0
    assert _parse_error_code(b"-1,{},JointMovJ(10.0,0.0,0.0,0.0,0.0,0.0);") == -1


def test_parse_error_code_raises_on_unrecognisable_response():
    with pytest.raises(Cr5ProtocolError):
        _parse_error_code(b"esto no es una respuesta del CR5")


def test_extract_last_frame_finds_the_frame_at_the_start_of_the_buffer():
    frame = _build_frame([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert _extract_last_frame(frame) == frame


def test_extract_last_frame_prefers_the_most_recent_of_two_frames():
    older = _build_frame([0.0] * 6)
    newer = _build_frame([90.0, -45.0, 0.0, 0.0, 0.0, 0.0])
    assert _extract_last_frame(older + newer) == newer


def test_extract_last_frame_raises_if_the_magic_value_is_never_found():
    with pytest.raises(Cr5ProtocolError):
        _extract_last_frame(b"\x00" * _FRAME_LENGTH)


# ---------------------------------------------------------------------
# Cr5CommandSocket -- contra un servidor de mentira real (dashboard/motion)
# ---------------------------------------------------------------------


def test_send_command_returns_the_error_code_from_a_dobot_style_response():
    received = {}

    def _respond(conn: socket.socket) -> None:
        received["command"] = conn.recv(1024)
        conn.sendall(b"0,{},EnableRobot();")

    port, thread = _serve_once(_respond)
    client = Cr5CommandSocket("127.0.0.1", port, timeout=1.0)
    client.connect()
    try:
        error_code = client.send_command("EnableRobot()")
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert received["command"] == b"EnableRobot()"
    assert error_code == 0


def test_send_command_surfaces_a_non_zero_error_code():
    def _respond(conn: socket.socket) -> None:
        conn.recv(1024)
        conn.sendall(b"-2,{},EnableRobot();")

    port, thread = _serve_once(_respond)
    client = Cr5CommandSocket("127.0.0.1", port, timeout=1.0)
    client.connect()
    try:
        error_code = client.send_command("EnableRobot()")
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert error_code == -2


def test_query_connects_by_itself_if_not_connected_yet():
    # No se llama a client.connect() -- query()/send_command() deben
    # conectar solos (ver docstring de Cr5CommandSocket, corregido 07/09).
    def _respond(conn: socket.socket) -> None:
        conn.recv(1024)
        conn.sendall(b"0,{},EnableRobot();")

    port, thread = _serve_once(_respond)
    client = Cr5CommandSocket("127.0.0.1", port, timeout=1.0)
    try:
        error_code = client.send_command("EnableRobot()")
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert error_code == 0


def test_query_reconnects_and_retries_once_after_a_dead_connection():
    # Primera conexión: se acepta el comando pero el servidor cierra SIN
    # responder -- simula una conexión que ya estaba muerta (idle
    # demasiado tiempo, reset a mitad de una ráfaga...). query() debe
    # reconectar (segunda conexión) y reintentar el MISMO comando antes de
    # rendirse -- ver docstring de Cr5CommandSocket sobre por qué esto es
    # seguro (todos los comandos del protocolo son idempotentes).
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(2)
    port = server.getsockname()[1]

    def _run() -> None:
        dead_conn, _ = server.accept()
        dead_conn.recv(1024)
        dead_conn.close()
        live_conn, _ = server.accept()
        live_conn.recv(1024)
        live_conn.sendall(b"0,{},EnableRobot();")
        live_conn.close()
        server.close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()

    client = Cr5CommandSocket("127.0.0.1", port, timeout=1.0)
    try:
        error_code = client.send_command("EnableRobot()")
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert error_code == 0


def test_query_gives_up_after_one_failed_retry():
    # Las DOS conexiones (la original y la única reconexión permitida)
    # mueren sin responder -- un solo reintento, no un bucle indefinido
    # (ver docstring de Cr5CommandSocket).
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(2)
    port = server.getsockname()[1]

    def _run() -> None:
        for _ in range(2):
            conn, _ = server.accept()
            conn.recv(1024)
            conn.close()
        server.close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()

    client = Cr5CommandSocket("127.0.0.1", port, timeout=1.0)
    try:
        with pytest.raises(Cr5ProtocolError):
            client.send_command("EnableRobot()")
    finally:
        client.close()
    thread.join(timeout=1.0)


def test_send_command_without_connect_raises() -> None:
    client = Cr5CommandSocket("127.0.0.1", 1, timeout=0.1)
    with pytest.raises(Cr5ProtocolError):
        client.send_command("EnableRobot()")


# ---------------------------------------------------------------------
# Cr5RealtimeSocket -- stream binario, no comandos
# ---------------------------------------------------------------------


def test_read_joint_angles_deg_extracts_q_actual_from_a_single_frame():
    angles = [10.0, -20.5, 30.25, 0.0, 90.0, -45.0]

    def _respond(conn: socket.socket) -> None:
        conn.sendall(_build_frame(angles))

    port, thread = _serve_once(_respond)
    client = Cr5RealtimeSocket("127.0.0.1", port, timeout=1.0)
    client.connect()
    try:
        result = client.read_joint_angles_deg()
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert result == pytest.approx(angles)


def test_read_joint_angles_deg_skips_a_stale_frame_sent_first():
    stale = _build_frame([0.0] * 6)
    fresh = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    def _respond(conn: socket.socket) -> None:
        conn.sendall(stale)
        conn.sendall(_build_frame(fresh))

    port, thread = _serve_once(_respond)
    client = Cr5RealtimeSocket("127.0.0.1", port, timeout=1.0)
    client.connect()
    # Deja tiempo a que las DOS tramas ya hayan llegado por loopback antes
    # de leer -- lo que este test valida es que, habiendo dos disponibles,
    # se queda con la última; no depende de en qué recv() exacto caiga cada
    # una (eso ya lo cubre _extract_last_frame por separado).
    time.sleep(0.05)
    try:
        result = client.read_joint_angles_deg()
    finally:
        client.close()
    thread.join(timeout=1.0)

    assert result == pytest.approx(fresh)
