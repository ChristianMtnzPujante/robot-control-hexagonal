"""Genera la guía de formatos YAML (una tabla por sección) a partir de los
MISMOS esquemas con los que se validan los ficheros: si un campo cambia en
el código, cambia en la guía. La guía vive en el vault
(`obsidian-vault/Arquitectura/Guía de formatos YAML.md`) y un test
comprueba que está al día.

Uso:
    ros2 run commander cell_guide            # la imprime
    ros2 run commander cell_guide --write    # actualiza la nota del vault
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List

from .compile import CELL, CELL_ROBOT, CELL_SCENE, CELL_SIMULATOR, CELL_TOOL
from .elements import ROBOT, ROBOT_REAL, ROBOT_SIM, TOOL, TOOL_GRASP, TOOL_MOUNT, TOOL_PADS, TOOL_REAL, TOOL_SIM
from .node_format import NODE_PARAMETER, NODE_TOPIC, NODE_TYPE
from .scene_format import BODY, OBSTACLE, PLANE, SCENE
from .schema import Section

GUIDE_PATH = Path(__file__).resolve().parents[4] / "obsidian-vault" / "Arquitectura" / "Guía de formatos YAML.md"

GROUPS = (
    ("Elementos del mundo", "Uno por fichero en `descriptions/`. No saben de ROS: un robot o una escena se reutilizan en varias células.",
     (ROBOT, ROBOT_SIM, ROBOT_REAL, TOOL, TOOL_MOUNT, TOOL_GRASP, TOOL_PADS, TOOL_SIM, TOOL_REAL, SCENE, BODY, OBSTACLE, PLANE)),
    ("Composición", "La célula referencia a los elementos y dice cómo se usan. Compilarla (`compile_cell`) resuelve las referencias, valida cada pieza y las reglas entre piezas.",
     (CELL, CELL_ROBOT, CELL_TOOL, CELL_SCENE, CELL_SIMULATOR)),
    ("Tipos de nodo", "La interfaz de cada nodo ROS, en su propio paquete. La usa el grafo de nodos de una célula (siguiente paso).",
     (NODE_TYPE, NODE_PARAMETER, NODE_TOPIC)),
)

_HEADER = """---
tags: [arquitectura, referencia, generado]
---

# Guía de formatos YAML

> [!warning] Nota GENERADA, no editar a mano
> Sale de los esquemas de `src/commander/commander/cell/` con
> `ros2 run commander cell_guide --write`. Un test falla si se queda
> atrás respecto al código. Para cambiar un campo, cámbialo en su esquema.

Tres clases de YAML que no se mezclan (decisión del 01/10, ver
[[Decisiones de Diseño Clave]]): los **elementos del mundo**, la
**composición** (la célula) y los **tipos de nodo**. Cómo encajan: ver
[[Células y Escenarios]]. «Obligatorio» puede ser una condición (p. ej. «si
`box`»): la comprueba el lector, no la tabla.
"""


def _cell(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", " ")


def section_table(section: Section) -> List[str]:
    lines = [f"### {section.title}", "", f"`{section.location}`", ""]
    if section.doc:
        lines += [section.doc, ""]
    lines += ["| Campo | Tipo | Obligatorio | Por defecto | Qué es |", "| --- | --- | --- | --- | --- |"]
    for field in section.fields:
        required = "sí" if field.required is True else "no" if field.required is False else field.required
        lines.append(
            f"| `{field.name}` | {_cell(field.kind)} | {_cell(required)} | "
            f"{_cell(field.default) if field.default else ''} | {_cell(field.doc)} |"
        )
    return lines + [""]


def render(groups: Iterable = GROUPS) -> str:
    lines = [_HEADER]
    for title, doc, sections in groups:
        lines += [f"## {title}", "", doc, ""]
        for section in sections:
            lines += section_table(section)
    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--write", action="store_true", help=f"Escribe la guía en {GUIDE_PATH}.")
    args = parser.parse_args()
    text = render()
    if args.write:
        GUIDE_PATH.write_text(text)
        print(f"Guía escrita en {GUIDE_PATH}")
    else:
        print(text)


if __name__ == "__main__":
    main()
