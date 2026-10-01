"""Cliente manual de `Commander`: en cada momento enseña las tools
disponibles, EXACTAMENTE como las vería un LLM (`ToolBox.list_tools`), y
deja que una persona elija cuál llamar y con qué opciones. Es el ensayo de
control por LLM con una persona decidiendo.

Lo que ve el LLM, y nada más, sale de `ToolBox`: la lista de tools (con
sus opciones como `enum`) y el JSON de cada resultado, errores incluidos.
Tras cada llamada, la consola dice qué tools han aparecido y desaparecido
-- lo que el servidor MCP notificará con `tools/list_changed`. La línea de
estado de arriba es solo para la persona.

Uso:
    ros2 run commander cell_console
    ros2 run commander cell_console --cell mesa_cubo --open

    En el menú: un número elige una tool; `j` enseña el JSON de las tools
    tal cual; `q` sale (cerrando todo lo abierto).
"""

from __future__ import annotations

import argparse
import json
from typing import Any, Callable, Dict, List, Optional

from .cell_manager import CellManager
from .tools import ToolBox


def _signature(tool: Dict[str, Any]) -> str:
    properties = tool["inputSchema"]["properties"]
    if not properties:
        return tool["name"]
    parts = []
    for parameter, spec in properties.items():
        kind = " | ".join(spec["enum"]) if "enum" in spec else "número" if spec["type"] == "number" else "texto"
        parts.append(f"{parameter}: {kind}")
    return f'{tool["name"]}({", ".join(parts)})'


class Console:
    def __init__(self, toolbox: ToolBox, read: Callable[[str], str] = input, write: Callable[[str], None] = print):
        self.toolbox = toolbox
        self.read = read
        self.write = write

    def run(self) -> None:
        previous: Optional[List[str]] = None
        try:
            while True:
                tools = self.toolbox.list_tools()
                names = [tool["name"] for tool in tools]
                self._show_status()
                if previous is not None:
                    self._show_changes(previous, names)
                previous = names
                self.write("Tools disponibles (lo que vería el LLM):")
                for number, tool in enumerate(tools, 1):
                    self.write(f"  {number:2d}. {_signature(tool)} — {tool['description']}")
                choice = self.read("Número de tool · j: JSON de las tools · q: salir > ").strip().lower()
                if choice in ("q", "salir"):
                    return
                if choice == "j":
                    self.write(json.dumps(tools, ensure_ascii=False, indent=2))
                    continue
                if not choice.isdigit() or not 1 <= int(choice) <= len(tools):
                    self.write("  (elige un número de la lista)")
                    continue
                tool = tools[int(choice) - 1]
                arguments = self._ask_arguments(tool)
                if arguments is None:
                    self.write("  (cancelado)")
                    continue
                self.write(f"→ {tool['name']}({json.dumps(arguments, ensure_ascii=False)})")
                result = self.toolbox.call(tool["name"], arguments)
                self.write(json.dumps(result, ensure_ascii=False, indent=2))
        finally:
            self.toolbox.close()

    def _ask_arguments(self, tool: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        arguments = {}
        for parameter, spec in tool["inputSchema"]["properties"].items():
            if "enum" in spec:
                options = spec["enum"]
                listing = "  ".join(f"{i}) {value}" for i, value in enumerate(options, 1))
                answer = self.read(f"  {parameter}: {listing} > ").strip()
                if not answer:
                    return None
                if answer.isdigit() and 1 <= int(answer) <= len(options):
                    answer = options[int(answer) - 1]
                arguments[parameter] = answer  # si no es una opción, lo dirá la propia tool
            else:
                answer = self.read(f"  {parameter} ({spec.get('description', 'texto')}) > ").strip()
                if not answer:
                    return None
                if spec["type"] == "number":
                    try:
                        answer = float(answer.replace(",", "."))  # admite coma decimal
                    except ValueError:
                        pass  # si no es un número, lo dirá la propia tool
                arguments[parameter] = answer
        return arguments

    def _show_status(self) -> None:
        active = self.toolbox.active
        if active is None:
            self.write("\n══ Sin célula activa")
            return
        info = self.toolbox.manager.describe(active)
        world = info["world"]
        if world["held_body"]:
            hands = f'sujeta "{world["held_body"]}"'
        elif world["holding"]:
            hands = "sujeta algo"
        elif world["holding"] is False:
            hands = "vacía"
        else:
            hands = "sin pinza o sin leer"
        bodies = "  ".join(
            f'{name}({", ".join(f"{v:+.3f}" for v in body["position"])}; {body["source"]})'
            for name, body in world["bodies"].items()
        )
        self.write(f'\n══ Célula activa: {active} ({info["target"]}) · pinza: {hands}   [solo para ti]')
        self.write(f"   {bodies}")

    def _show_changes(self, previous: List[str], current: List[str]) -> None:
        added = [name for name in current if name not in previous]
        removed = [name for name in previous if name not in current]
        if added or removed:
            parts = [f"+{name}" for name in added] + [f"−{name}" for name in removed]
            self.write(f"   Cambian las tools (tools/list_changed): {' '.join(parts)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", help="Crea esta célula al empezar (nombre en descriptions/cells/ o ruta).")
    parser.add_argument("--open", action="store_true", help="Y la abre.")
    args = parser.parse_args()

    toolbox = ToolBox(CellManager())
    if args.cell:
        result = toolbox.call("create_cell", {"source": args.cell})
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if result["ok"] and args.open:
            print(json.dumps(toolbox.call("open_cell", {"cell": result["cell"]}), ensure_ascii=False, indent=2))
    Console(toolbox).run()


if __name__ == "__main__":
    main()
