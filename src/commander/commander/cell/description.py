"""Descripción declarativa de una célula de trabajo: qué robot, qué
herramientas lleva montadas, con qué cinemática se calcula, qué posturas
tienen nombre y qué hay en la escena. Es la fuente única de la que se
construye la célula, en cualquiera de los dos modos de ejecución
(directo, en este proceso; o por ROS, con sesiones -- fase 2) y contra
cualquiera de los dos destinos (simulación o robot real).

Solo datos y validación: no sabe qué es CoppeliaSim ni el CR5. Los
modelos (`robot.model`, `tools[].model`) son claves del catálogo
(`catalog.py`), que es quien sabe cómo se construye cada uno. Se escribe a
mano en YAML (`scenarios/*.yaml`, ver `loader.py`) o se construye en
código.

Ver la decisión de diseño del 01/10 en el vault: "Decisiones de Diseño
Clave" (descripción declarativa + dos modos de ejecución).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

from shared_kernel import Scene

TARGETS = ("sim", "real")
KINEMATICS = ("poe", "ga")


class InvalidCellError(ValueError):
    """La descripción de la célula está incompleta o es incoherente."""


@dataclass(frozen=True)
class RobotSpec:
    """`model` es una clave del catálogo (p. ej. "cr5"). `target` dice
    dónde está el robot: "sim" (CoppeliaSim) o "real". `host` solo cuenta
    en real. `initial_posture` es la postura (por nombre) en la que se crea
    el robot en simulación; en real no se mueve nada al abrir la célula."""

    model: str
    target: str = "sim"
    host: Optional[str] = None
    initial_posture: Optional[str] = None


@dataclass(frozen=True)
class ToolSpec:
    """Una herramienta montada en el robot. `grasp_offset` es la distancia
    de la brida al centro de lo que agarra (metros, a lo largo del eje de
    la herramienta). En simulación es opcional: sale del modelo. En real
    es OBLIGATORIO, porque depende del montaje (acoplador incluido) y hay
    que medirlo."""

    model: str
    grasp_offset: Optional[float] = None


@dataclass(frozen=True)
class SimulatorSpec:
    port: int = 23000
    step_pause_seconds: float = 0.04


@dataclass(frozen=True)
class CellDescription:
    """`postures` son configuraciones articulares con nombre, en GRADOS y
    en el orden de joints del modelo del robot. `scene` es el estado
    inicial del mundo (cuerpos, puntos con nombre, obstáculos...)."""

    name: str
    robot: RobotSpec
    tools: Tuple[ToolSpec, ...] = ()
    kinematics: str = "poe"
    simulator: SimulatorSpec = SimulatorSpec()
    postures: Dict[str, Tuple[float, ...]] = field(default_factory=dict)
    scene: Scene = field(default_factory=Scene.empty)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Comprobaciones que no dependen del catálogo (eso lo hace
        `catalog.py` al resolver los modelos)."""
        if not self.name:
            raise InvalidCellError("la célula necesita un nombre")
        if self.robot.target not in TARGETS:
            raise InvalidCellError(
                f'robot.target "{self.robot.target}" no es válido: {", ".join(TARGETS)}'
            )
        if self.robot.target == "real" and not self.robot.host:
            raise InvalidCellError('robot.target "real" necesita robot.host (la IP del controlador)')
        if self.kinematics not in KINEMATICS:
            raise InvalidCellError(
                f'kinematics "{self.kinematics}" no es válida: {", ".join(KINEMATICS)}'
            )
        if len(self.tools) > 1:
            raise InvalidCellError("de momento, una sola herramienta por célula")
        if self.robot.target == "real":
            for tool in self.tools:
                if tool.grasp_offset is None:
                    raise InvalidCellError(
                        f'la herramienta "{tool.model}" necesita grasp_offset MEDIDO para '
                        "usarla en el robot real: depende del montaje, no del modelo"
                    )
        if self.robot.initial_posture and self.robot.initial_posture not in self.postures:
            raise InvalidCellError(
                f'robot.initial_posture "{self.robot.initial_posture}" no está en postures'
            )

    @property
    def tool(self) -> Optional[ToolSpec]:
        return self.tools[0] if self.tools else None

    def with_target(self, target: str, host: Optional[str] = None) -> "CellDescription":
        """La misma célula contra otro destino (p. ej. probar en sim lo que
        el YAML describe para el real, o al revés). Vuelve a validar."""
        robot = replace(self.robot, target=target, host=host or self.robot.host)
        return replace(self, robot=robot)
