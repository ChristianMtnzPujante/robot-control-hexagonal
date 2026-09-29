"""Tests de Robotiq2FGripperAdapter contra un socket de comandos de mentira.

Qué validan y qué NO: validan que empaquetamos los bytes como dice el manual
de Robotiq y que mandamos los comandos Modbus que la pinza real SÍ contesta
(vía 127.0.0.1:60000, verificada el 29/09 desde script). NO sustituyen la
prueba contra el hardware.
"""

from typing import List, Tuple

import pytest

from robot_node.adapters._cr5_protocol import Cr5ProtocolError
from robot_node.adapters.robotiq_2f_adapter import Robotiq2FGripperAdapter


class FakeCommandSocket:
    """Imita Cr5CommandSocket.query: registra lo que se manda y devuelve
    respuestas preparadas."""

    def __init__(self, responses=None):
        self.sent: List[str] = []
        self._responses = dict(responses or {})

    def query(self, command: str) -> Tuple[int, str]:
        self.sent.append(command)
        for prefix, response in self._responses.items():
            if command.startswith(prefix):
                return response
        return (0, "")


def _adapter(responses=None):
    socket = FakeCommandSocket({"ModbusCreate": (0, "0"), **(responses or {})})
    return Robotiq2FGripperAdapter(socket), socket


def test_prepares_the_tool_485_and_opens_the_master_through_port_60000():
    adapter, socket = _adapter()
    adapter.set_opening(1.0)
    assert socket.sent[:3] == [
        "SetToolMode(1)",
        'SetTool485(115200,"N",1)',
        'ModbusCreate("127.0.0.1",60000,9,1)',
    ]


def test_does_not_use_modbus_rtu_create():
    # ModbusRTUCreate no llega a la brida: la pinza no contestó nunca con él.
    adapter, socket = _adapter()
    adapter.set_opening(1.0)
    assert not any(c.startswith("ModbusRTUCreate") for c in socket.sent)


def test_a_failed_tool_485_setup_stops_before_creating_the_master():
    adapter, socket = _adapter({"SetTool485": (-1, "")})
    with pytest.raises(Cr5ProtocolError):
        adapter.set_opening(1.0)
    assert not any(c.startswith("ModbusCreate") for c in socket.sent)


def test_the_master_is_created_only_once():
    adapter, socket = _adapter()
    adapter.set_opening(1.0)
    adapter.set_opening(0.0)
    assert sum(1 for c in socket.sent if c.startswith("ModbusCreate")) == 1


def test_closing_packs_position_255_with_ract_and_rgto():
    adapter, socket = _adapter()
    adapter.set_opening(1.0)
    # 2304 = 0x0900 -> rACT(bit0) + rGTO(bit3) en el byte 0; 255 = rPR;
    # 32896 = 0x8080 -> velocidad 128, fuerza 128.
    assert socket.sent[-1] == "SetHoldRegs(0,1000,3,{2304,255,32896},U16)"


def test_opening_packs_position_zero():
    adapter, socket = _adapter()
    adapter.set_opening(0.0)
    assert socket.sent[-1] == "SetHoldRegs(0,1000,3,{2304,0,32896},U16)"


def test_halfway_rounds_to_the_middle_of_the_range():
    adapter, socket = _adapter()
    adapter.set_opening(0.5)
    assert socket.sent[-1] == "SetHoldRegs(0,1000,3,{2304,128,32896},U16)"


def test_speed_and_force_are_packed_in_the_third_register():
    adapter, socket = _adapter()
    adapter.set_opening(1.0, speed=0, force=255)
    assert socket.sent[-1] == "SetHoldRegs(0,1000,3,{2304,255,255},U16)"


def test_activation_resets_first_to_get_a_rising_edge_on_ract():
    # byte0 = 0x00 -> sin activar
    adapter, socket = _adapter({"GetHoldRegs": (0, "0,0,768")})
    adapter.activate()
    writes = [c for c in socket.sent if c.startswith("SetHoldRegs")]
    assert writes == [
        "SetHoldRegs(0,1000,3,{0,0,0},U16)",
        "SetHoldRegs(0,1000,3,{256,0,0},U16)",  # 256 = 0x0100 -> rACT=1
    ]


def test_activation_does_nothing_if_already_active():
    # Lectura real del 29/09: 0x3100 = gACT + gSTA=3, gFLT=0x09 (sin
    # comunicación, fallo menor). Re-activar soltaría lo agarrado.
    adapter, socket = _adapter({"GetHoldRegs": (0, "12544,2304,768")})
    adapter.activate()
    assert not any(c.startswith("SetHoldRegs") for c in socket.sent)


def test_activation_resets_a_major_fault_even_if_active():
    # gFLT = 0x0E (sobrecorriente): solo se borra con un reset
    adapter, socket = _adapter({"GetHoldRegs": (0, "12544,3584,768")})
    adapter.activate()
    writes = [c for c in socket.sent if c.startswith("SetHoldRegs")]
    assert writes[0] == "SetHoldRegs(0,1000,3,{0,0,0},U16)"
    assert len(writes) == 2


def test_out_of_range_opening_is_rejected_before_touching_the_robot():
    adapter, socket = _adapter()
    with pytest.raises(ValueError):
        adapter.set_opening(1.5)
    assert socket.sent == []


def test_get_state_reads_the_status_block():
    adapter, socket = _adapter({"GetHoldRegs": (0, "6912,0,32768")})
    state = adapter.get_state()
    assert socket.sent[-1] == "GetHoldRegs(0,2000,3)"
    # 6912 = 0x1B00 -> byte0 = 0x1B = gACT=1, gGTO=1, gSTA=1 -> sin activar del todo
    assert state.activated is False
    assert state.opening == pytest.approx(128 / 255.0)  # gPO = 0x80


def test_get_state_decodes_activation_and_object_detection():
    # byte0 = 0b11111001 = 0xF9 -> gOBJ=3 (llegó, sin objeto), gSTA=3 (activada)
    adapter, _ = _adapter({"GetHoldRegs": (0, "63744,0,65280")})
    state = adapter.get_state()
    assert state.activated is True
    assert state.holding_object is False
    assert state.opening == pytest.approx(1.0)

    # byte0 = 0b10111001 = 0xB9 -> gOBJ=2 (paró al cerrar: OBJETO)
    adapter, _ = _adapter({"GetHoldRegs": (0, "47360,0,32768")})
    assert adapter.get_state().holding_object is True


def test_get_state_surfaces_the_fault_code():
    # reg1 = 0x0700 -> gFLT = 0x07 ("hay que activar antes de mandar nada")
    adapter, _ = _adapter({"GetHoldRegs": (0, "0,1792,0")})
    assert adapter.get_state().fault_code == 0x07


def test_fault_code_ignores_the_controller_fault_nibble():
    # byte2 = 0x49 -> kFLT = 0x4 (bits 4-7), gFLT = 0x9 (bits 0-3)
    adapter, _ = _adapter({"GetHoldRegs": (0, "0,18688,0")})
    assert adapter.get_state().fault_code == 0x09


def test_a_failed_read_explains_that_minus_one_is_ambiguous():
    adapter, _ = _adapter({"GetHoldRegs": (-1, "")})
    with pytest.raises(Cr5ProtocolError) as error:
        adapter.get_state()
    assert "485A/485B" in str(error.value)


def test_close_releases_the_master_and_allows_reopening():
    adapter, socket = _adapter()
    adapter.set_opening(1.0)
    adapter.close()
    assert socket.sent[-1] == "ModbusClose(0)"
    adapter.set_opening(0.0)
    assert sum(1 for c in socket.sent if c.startswith("ModbusCreate")) == 2


def test_close_without_ever_talking_does_nothing():
    adapter, socket = _adapter()
    adapter.close()
    assert socket.sent == []


def test_after_a_failure_nothing_else_is_sent_on_the_shared_socket():
    # Pinza ausente: GetHoldRegs -> -1. Las órdenes siguientes no deben
    # volver a ocupar el socket del brazo (~0,5 s por cada -1).
    adapter, socket = _adapter({"GetHoldRegs": (-1, "")})
    with pytest.raises(Cr5ProtocolError):
        adapter.get_state()
    sent_before = len(socket.sent)
    for call in (adapter.get_state, lambda: adapter.set_opening(1.0), adapter.activate):
        with pytest.raises(Cr5ProtocolError) as error:
            call()
        assert "no disponible" in str(error.value)
    assert len(socket.sent) == sent_before


def test_a_failed_setup_also_marks_the_gripper_unavailable():
    adapter, socket = _adapter({"SetToolMode": (-1, "")})
    with pytest.raises(Cr5ProtocolError):
        adapter.set_opening(1.0)
    with pytest.raises(Cr5ProtocolError):
        adapter.set_opening(1.0)
    assert sum(1 for c in socket.sent if c.startswith("SetToolMode")) == 1


def test_close_clears_the_unavailable_mark_so_it_can_retry():
    adapter, socket = _adapter({"GetHoldRegs": (-1, "")})
    with pytest.raises(Cr5ProtocolError):
        adapter.get_state()
    adapter.close()
    with pytest.raises(Cr5ProtocolError) as error:
        adapter.get_state()  # vuelve a preguntar (y vuelve a fallar)
    assert "no disponible" not in str(error.value).split("--")[0]
    assert sum(1 for c in socket.sent if c.startswith("GetHoldRegs")) == 2


class _BrokenConnectionSocket(FakeCommandSocket):
    """La CONEXIÓN falla (no la pinza): query lanza en vez de devolver -1."""

    def __init__(self):
        super().__init__({"ModbusCreate": (0, "0")})
        self.broken = True

    def query(self, command: str) -> Tuple[int, str]:
        if self.broken and command.startswith("GetHoldRegs"):
            self.sent.append(command)
            raise Cr5ProtocolError("socket cerrado")
        return super().query(command)


def test_a_connection_failure_does_not_mark_the_gripper_unavailable():
    socket = _BrokenConnectionSocket()
    adapter = Robotiq2FGripperAdapter(socket)
    with pytest.raises(Cr5ProtocolError):
        adapter.get_state()
    socket.broken = False
    socket._responses["GetHoldRegs"] = (0, "12544,0,768")
    assert adapter.get_state().activated is True
