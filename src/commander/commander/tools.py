"""La "caja de herramientas": lo que un cliente -- hoy la consola manual,
mañana el servidor MCP y un LLM -- ve y puede pedir a `Commander`.

Dos métodos, nada más:

- `list_tools()`: las tools disponibles AHORA, en el formato de MCP
  (`name`, `description`, `inputSchema` con las opciones como `enum`).
  Cambia con el estado: crear o abrir una célula, coger algo... El servidor
  MCP avisará al cliente con `tools/list_changed`; la consola enseña qué
  ha aparecido y desaparecido.
- `call(name, arguments)`: ejecuta y devuelve SIEMPRE un resultado JSON,
  también si hay error (`{"ok": false, "error": ...}`): es lo que vería el
  LLM, que tiene que poder leerlo y corregirse.

Dos niveles de tools (ver "Roles del Commander" en el vault):
configuración (células) y operación (sobre la célula ACTIVA). Con varias
células abiertas, `select_cell` cambia la activa: así cada tool tiene
opciones fijas, sin depender de qué célula se elija en la misma llamada.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from .cell import InvalidCellError
from .cell_manager import CellManager, check_value


def _tool(name: str, description: str, properties: Optional[Dict[str, Dict[str, Any]]] = None) -> Dict[str, Any]:
    properties = properties or {}
    return {
        "name": name,
        "description": description,
        "inputSchema": {"type": "object", "properties": properties, "required": sorted(properties)},
    }


def _choice(values: Sequence[str], description: str) -> Dict[str, Any]:
    return {"type": "string", "enum": list(values), "description": description}


class ToolBox:
    def __init__(self, manager: CellManager):
        self.manager = manager
        self.active: Optional[str] = None

    # --- Qué se ofrece ahora -------------------------------------------------------

    def list_tools(self) -> List[Dict[str, Any]]:
        cells = self.manager.list_cells()
        closed = [c["name"] for c in cells if not c["open"]]
        opened = [c["name"] for c in cells if c["open"]]
        tools = [
            _tool("list_cells", "Lista las células creadas, si están abiertas y cuál es la activa."),
            _tool("create_cell", "Crea (compila y valida, sin abrir) una célula a partir de su descripción.",
                  {"source": {"type": "string",
                              "description": "Nombre en descriptions/cells/ o ruta a un .yaml."}}),
        ]
        if closed:
            tools.append(_tool("open_cell", "Abre una célula creada (construye su mundo) y la deja activa.",
                               {"cell": _choice(closed, "Célula a abrir.")}))
        if opened:
            tools.append(_tool("close_cell", "Cierra una célula abierta.", {"cell": _choice(opened, "Célula a cerrar.")}))
        if len(opened) > 1:
            tools.append(_tool("select_cell", "Cambia la célula activa.", {"cell": _choice(opened, "Célula activa.")}))
        if self.active is not None:
            tools.append(_tool("get_world", f'Estado del mundo de "{self.active}": cuerpos (con origen y '
                                            "antigüedad de cada dato), puntos, pinza y brazo."))
            for operation in self.manager.describe(self.active)["operations"]:
                tools.append(_tool(operation["name"], f'{operation["description"]} (célula "{self.active}")',
                                   operation["parameters"]))
        return tools

    # --- Pedir algo ----------------------------------------------------------------

    def call(self, name: str, arguments: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        arguments = dict(arguments or {})
        tools = {tool["name"]: tool for tool in self.list_tools()}
        if name not in tools:
            return {"ok": False, "error": f'la tool "{name}" no está disponible ahora', "available": sorted(tools)}
        schema = tools[name]["inputSchema"]
        problem = self._check_arguments(schema, arguments)
        if problem:
            return {"ok": False, "error": problem, "inputSchema": schema}
        try:
            return self._dispatch(name, arguments)
        except InvalidCellError as error:
            return {"ok": False, "error": str(error)}

    @staticmethod
    def _check_arguments(schema: Dict[str, Any], arguments: Dict[str, Any]) -> Optional[str]:
        properties = schema["properties"]
        unknown = sorted(set(arguments) - set(properties))
        missing = sorted(set(schema["required"]) - set(arguments))
        if unknown or missing:
            return f"argumentos incorrectos (sobran: {unknown or 'nada'}; faltan: {missing or 'nada'})"
        for parameter, value in arguments.items():
            problem = check_value(properties[parameter], value)
            if problem:
                return f"{parameter}: {problem}"
        return None

    def _dispatch(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        manager = self.manager
        if name == "list_cells":
            return {"ok": True, "cells": manager.list_cells(), "active": self.active}
        if name == "create_cell":
            description = manager.create_cell(arguments["source"])
            info = manager.describe(description.name)
            return {"ok": True, "cell": description.name, "target": info["target"],
                    "capabilities": [c["name"] for c in info["capabilities"]]}
        if name == "open_cell":
            manager.open_cell(arguments["cell"])
            self.active = arguments["cell"]
            return {"ok": True, "active": self.active,
                    "operations": [op["name"] for op in manager.describe(self.active)["operations"]]}
        if name == "close_cell":
            manager.close_cell(arguments["cell"])
            if self.active == arguments["cell"]:
                still_open = [c["name"] for c in manager.list_cells() if c["open"]]
                self.active = still_open[0] if still_open else None
            return {"ok": True, "active": self.active}
        if name == "select_cell":
            self.active = arguments["cell"]
            return {"ok": True, "active": self.active}
        if name == "get_world":
            return {"ok": True, "cell": self.active, "world": manager.describe(self.active)["world"]}
        return manager.execute(self.active, name, arguments)

    def close(self) -> None:
        self.manager.close_all()
        self.active = None
