"""La célula COMPILADA: el robot, la herramienta y la escena ya resueltos
(no nombres por buscar), más cómo se usan en esta célula (destino sim o
real, cinemática, posturas de la tarea). Es lo que consumen los modos de
ejecución -- el directo (`direct.py`) y, en la fase 2, el de ROS.

Se obtiene compilando un fichero de `descriptions/cells/` (`compile.py`),
que referencia a un robot, una herramienta y una escena, cada uno en su
propio formato. También se puede construir en código.

`validate()` comprueba las reglas ENTRE piezas, que ningún formato puede
comprobar solo: que la herramienta tenga montaje previsto en ese robot,
que cada postura tenga un valor por joint, que el real tenga IP y una
distancia de agarre medida...

Ver las decisiones del 01/10 en el vault ("Decisiones de Diseño Clave").
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

from shared_kernel import Scene

from .elements import RobotModel, ToolModel
from .errors import InvalidCellError

TARGETS = ("sim", "real")
KINEMATICS = ("poe", "ga")

__all__ = ["CellDescription", "InvalidCellError", "RobotSpec", "SimulatorSpec", "ToolSpec"]


@dataclass(frozen=True)
class RobotSpec:
    """El robot `model` usado en esta célula. `target`: "sim" (CoppeliaSim)
    o "real". `host` solo cuenta en real. `initial_posture` es la postura en
    la que se crea en simulación; en real no se mueve nada al abrir."""

    model: RobotModel
    target: str = "sim"
    host: Optional[str] = None
    initial_posture: Optional[str] = None
    speed_factor: Optional[int] = None  # solo real: velocidad global en %, 1-100


@dataclass(frozen=True)
class ToolSpec:
    """La herramienta `model` montada en esta célula. `grasp_offset` es la
    distancia de agarre MEDIDA en este montaje; sin ella se usa la del
    modelo, que solo vale en simulación."""

    model: ToolModel
    grasp_offset: Optional[float] = None

    @property
    def effective_grasp_offset(self) -> float:
        return self.grasp_offset if self.grasp_offset is not None else self.model.grasp_offset


@dataclass(frozen=True)
class SimulatorSpec:
    port: int = 23000
    step_pause_seconds: float = 0.04


@dataclass(frozen=True)
class CellDescription:
    """`postures` son TODAS las de la célula: las del modelo del robot más
    las propias de la tarea (grados, en el orden de joints del robot)."""

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
        robot = self.robot.model
        if not self.name:
            raise InvalidCellError("la célula necesita un nombre")
        if self.robot.target not in TARGETS:
            raise InvalidCellError(f'robot.target "{self.robot.target}" no es válido: {", ".join(TARGETS)}')
        if self.robot.target == "real":
            if not self.robot.host:
                raise InvalidCellError('robot.target "real" necesita robot.host (la IP del controlador)')
            if not robot.real_adapter:
                raise InvalidCellError(f'el robot "{robot.name}" no tiene sección "real": solo simulación')
        if self.robot.speed_factor is not None and not 1 <= self.robot.speed_factor <= 100:
            raise InvalidCellError(f"robot.speed_factor debe estar entre 1 y 100, no {self.robot.speed_factor}")
        if self.kinematics not in KINEMATICS:
            raise InvalidCellError(f'kinematics "{self.kinematics}" no es válida: {", ".join(KINEMATICS)}')
        if len(self.tools) > 1:
            raise InvalidCellError("de momento, una sola herramienta por célula")
        for tool in self.tools:
            if robot.name not in tool.model.mounts:
                raise InvalidCellError(
                    f'la herramienta "{tool.model.name}" no tiene montaje previsto en el robot "{robot.name}"'
                )
            if self.robot.target == "real":
                if tool.grasp_offset is None:
                    raise InvalidCellError(
                        f'la herramienta "{tool.model.name}" necesita grasp_offset MEDIDO en la célula '
                        "para usarla en el robot real: depende del montaje, no del modelo"
                    )
                if not tool.model.real_adapter:
                    raise InvalidCellError(f'la herramienta "{tool.model.name}" no tiene sección "real"')
        for name, values in self.postures.items():
            if len(values) != len(robot.joint_names):
                raise InvalidCellError(
                    f'la postura "{name}" tiene {len(values)} valores y el robot {len(robot.joint_names)} joints'
                )
        if self.robot.initial_posture and self.robot.initial_posture not in self.postures:
            raise InvalidCellError(f'robot.initial_posture "{self.robot.initial_posture}" no está en las posturas')

    @property
    def tool(self) -> Optional[ToolSpec]:
        return self.tools[0] if self.tools else None

    def with_target(self, target: str, host: Optional[str] = None) -> "CellDescription":
        """La misma célula contra otro destino (p. ej. probar en sim lo que
        se describió para el real). Vuelve a validar."""
        return replace(self, robot=replace(self.robot, target=target, host=host or self.robot.host))

    def with_speed_factor(self, speed_factor: Optional[int]) -> "CellDescription":
        return replace(self, robot=replace(self.robot, speed_factor=speed_factor))
