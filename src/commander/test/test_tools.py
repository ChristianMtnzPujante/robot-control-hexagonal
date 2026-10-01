"""Tests de la caja de herramientas (lo que verá un LLM) y de la consola
manual, con la ejecución falsa de `test_cell_manager`: sin CoppeliaSim."""

import json

import pytest

from commander.cell_console import Console
from commander.cell_manager import CellManager
from commander.tools import ToolBox

from cell_fixtures import PADS, TOOL
from test_cell_manager import Clock, FakeRuntime


@pytest.fixture
def setup(tree):
    """(caja de herramientas, descripción de una célula que se puede crear)."""
    box = ToolBox(CellManager(open_runtime=FakeRuntime(), clock=Clock()))
    return box, tree(tool=dict(TOOL, grasp={"offset": 0.1, "pads": PADS}))


@pytest.fixture
def toolbox(setup):
    return setup[0]


@pytest.fixture
def opened(setup):
    box, description = setup
    box.manager.create_cell(description)
    box.call("open_cell", {"cell": "prueba"})
    return box


def _names(box):
    return [tool["name"] for tool in box.list_tools()]


def test_before_any_cell_only_configuration_tools_are_offered(toolbox):
    assert _names(toolbox) == ["list_cells", "create_cell"]


def test_tools_follow_the_mcp_format_with_options_as_enums(opened):
    pick = next(t for t in opened.list_tools() if t["name"] == "pick")
    assert pick["inputSchema"] == {
        "type": "object",
        "properties": {"body": {"type": "string", "enum": ["cubo"], "description": "cuerpo a coger"}},
        "required": ["body"],
    }
    json.dumps(opened.list_tools())  # todo serializable


def test_opening_a_cell_makes_it_active_and_offers_its_operations(setup):
    toolbox, description = setup
    toolbox.manager.create_cell(description)
    assert "open_cell" in _names(toolbox)
    result = toolbox.call("open_cell", {"cell": "prueba"})
    assert result["ok"] and toolbox.active == "prueba"
    names = _names(toolbox)
    assert "open_cell" not in names and {"close_cell", "get_world", "pick", "move_to_posture"} <= set(names)


def test_pick_then_place_changes_the_offered_tools(opened):
    assert opened.call("pick", {"body": "cubo"}) == {"ok": True, "message": 'sujeta "cubo"'}
    names = _names(opened)
    assert "place" in names and "pick" not in names
    result = opened.call("place", {"point": "destino"})
    assert result["ok"] and "pick" in _names(opened)


def test_calls_that_are_not_offered_or_badly_formed_come_back_as_errors(opened):
    not_now = opened.call("place", {"point": "destino"})
    assert not not_now["ok"] and "no está disponible ahora" in not_now["error"] and "pick" in not_now["available"]
    wrong_option = opened.call("pick", {"body": "mesa"})
    assert not wrong_option["ok"] and "no es una opción" in wrong_option["error"]
    missing = opened.call("pick", {})
    assert not missing["ok"] and "faltan: ['body']" in missing["error"]


def test_an_invalid_cell_comes_back_as_an_error_with_its_reason(toolbox):
    result = toolbox.call("create_cell", {"source": "no_existe"})
    assert not result["ok"] and "no encuentro" in result["error"]


def test_get_world_reports_bodies_with_their_origin(opened):
    world = opened.call("get_world")["world"]
    assert world["bodies"]["cubo"]["source"] == "escena_inicial" and world["holding"] is False


def test_closing_the_active_cell_leaves_no_active_cell(opened):
    assert opened.call("close_cell", {"cell": "prueba"}) == {"ok": True, "active": None}
    assert _names(opened) == ["list_cells", "create_cell", "open_cell"]


def test_the_console_shows_tools_runs_the_choice_and_announces_changes(setup):
    toolbox, description = setup
    toolbox.manager.create_cell(description)
    output = []
    script = iter([("tool", "open_cell"), ("opción", "1"), ("tool", "pick"), ("opción", "1"), ("tool", "q")])

    def read(prompt):
        kind, value = next(script)
        if kind == "tool" and value != "q":
            # el número con el que la consola acaba de listar esa tool
            line = next(l for l in reversed(output) if f". {value}" in l)
            return line.split(".")[0].strip()
        return value

    Console(toolbox, read=read, write=output.append).run()
    text = "\n".join(output)
    assert '→ open_cell({"cell": "prueba"})' in text
    assert '→ pick({"body": "cubo"})' in text and '"message": "sujeta \\"cubo\\""' in text
    assert "Cambian las tools (tools/list_changed): +place" in text
    assert toolbox.active is None  # al salir, todo cerrado
