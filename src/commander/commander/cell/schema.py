"""Mini-esquema de los ficheros YAML de `descriptions/`: cada sección
declara sus campos UNA vez (nombre, tipo, si es obligatorio, qué es), y de
esa misma declaración salen la validación estricta (claves desconocidas y
obligatorias) y la tabla de la guía (`guide.py`). Así la guía no puede
quedarse atrás respecto al código.

El tipo de cada valor lo comprueban los lectores con las utilidades de
abajo (`number`, `numbers`...), que dicen DÓNDE está el error
(`scene.bodies.cubo.size debería tener 3 números`).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple, Union

from .errors import InvalidCellError


@dataclass(frozen=True)
class Field:
    """`required`: True, False, o un texto con la condición (p. ej.
    "si shape: box") -- en ese caso lo comprueba el lector, no `check`."""

    name: str
    kind: str
    required: Union[bool, str]
    doc: str
    default: Optional[str] = None


@dataclass(frozen=True)
class Section:
    title: str
    location: str
    fields: Tuple[Field, ...]
    doc: str = ""

    @property
    def keys(self) -> Tuple[str, ...]:
        return tuple(field.name for field in self.fields)

    def check(self, data: Any, where: str) -> Dict[str, Any]:
        """Que sea un diccionario, sin claves desconocidas y con las
        obligatorias. Devuelve el diccionario."""
        data = mapping(data, where)
        unknown = sorted(set(data) - set(self.keys))
        if unknown:
            raise InvalidCellError(
                f'{where}: clave(s) desconocida(s) {", ".join(unknown)}. '
                f'Válidas: {", ".join(sorted(self.keys))}'
            )
        for field in self.fields:
            if field.required is True and data.get(field.name) is None:
                raise InvalidCellError(f'{where}: falta "{field.name}"')
        return data


# --- Utilidades de lectura con el sitio del error ------------------------------


def mapping(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise InvalidCellError(f"{where} debería ser un diccionario (clave: valor)")
    return value


def required(data: Dict[str, Any], key: str, where: str) -> Any:
    if data.get(key) is None:
        raise InvalidCellError(f'{where}: falta "{key}"')
    return data[key]


def text(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise InvalidCellError(f"{where} debería ser un texto no vacío, no {value!r}")
    return value


def number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidCellError(f"{where} debería ser un número, no {value!r}")
    return float(value)


def numbers(value: Any, where: str, length: Optional[int] = None) -> Tuple[float, ...]:
    if not isinstance(value, list):
        raise InvalidCellError(f"{where} debería ser una lista de números")
    if length is not None and len(value) != length:
        raise InvalidCellError(f"{where} debería tener {length} números, tiene {len(value)}")
    return tuple(number(v, f"{where}[{i}]") for i, v in enumerate(value))


def texts(value: Any, where: str) -> Tuple[str, ...]:
    if not isinstance(value, list):
        raise InvalidCellError(f"{where} debería ser una lista de textos")
    return tuple(text(v, f"{where}[{i}]") for i, v in enumerate(value))


def boolean(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise InvalidCellError(f"{where} debería ser true o false, no {value!r}")
    return value


def path_relative_to(value: Any, base_dir: Path, where: str, must_exist: bool = True) -> str:
    """Una ruta del YAML: `~` se expande y una ruta relativa lo es al
    fichero que la contiene (así `../../assets/...` funciona desde
    `descriptions/tools/`)."""
    path = Path(text(value, where)).expanduser()
    if not path.is_absolute():
        path = (base_dir / path).resolve()
    if must_exist and not path.exists():
        raise InvalidCellError(f"{where}: no existe {path}")
    return str(path)


def build(where: str, factory: Callable[[], Any]) -> Any:
    """Las primitivas del dominio validan sus medidas con ValueError: se
    reetiqueta con el sitio del fichero."""
    try:
        return factory()
    except InvalidCellError:
        raise
    except ValueError as error:
        raise InvalidCellError(f"{where}: {error}") from error
