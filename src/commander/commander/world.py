"""El modelo del mundo de una célula abierta: lo que `Commander` cree que
hay y en qué estado está, con DE DÓNDE sale cada dato y CUÁNDO. Es la
única fuente de verdad que consultan los demás (el servidor MCP para saber
qué tools ofrecer, más adelante el guardián y el supervisor del Bloque 7).

Se crea desde la escena inicial de la célula y se actualiza desde varias
fuentes que pueden contradecirse. Orden de confianza, de más a menos:

    simulador  > percepción > acción > escena inicial

con una regla más, necesaria en el robot real: **nuestras propias
acciones invalidan lo visto antes**, porque cambian el mundo (si la
percepción vio el cubo y luego lo movemos, esa observación ya es falsa).
En concreto, un dato nuevo se acepta si NO es más antiguo que el que hay
y, además, su fuente tiene igual o más rango, o es una acción. Así la
percepción posterior a un `place` corrige lo que esperábamos, y en
simulación la percepción nunca pisa la verdad del simulador.

Inmutable por dentro (la `Scene` lo es); cada cambio sustituye la escena y
avisa a quien se haya suscrito.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional

from shared_kernel import Body, GripperState, JointConfiguration, Pose, Scene

INITIAL = "escena_inicial"
ACTION = "accion"
PERCEPTION = "percepcion"
SIMULATOR = "simulador"
# Lectura del propio hardware (brazo, pinza): solo para su estado, que no
# compite con nada, así que no entra en el orden de confianza de los cuerpos.
HARDWARE = "hardware"
_RANK = {INITIAL: 0, ACTION: 1, PERCEPTION: 2, SIMULATOR: 3}


@dataclass(frozen=True)
class Provenance:
    source: str
    timestamp: float


@dataclass(frozen=True)
class WorldChange:
    """Qué ha cambiado: `kind` es "body", "robot" o "gripper"; `name` el
    cuerpo, si aplica."""

    kind: str
    source: str
    name: Optional[str] = None


def accepts(current: Provenance, new: Provenance) -> bool:
    """La regla de arriba: no más antiguo, y fuente de igual o más rango o
    una acción propia."""
    if new.timestamp < current.timestamp:
        return False
    return new.source == ACTION or _RANK[new.source] >= _RANK[current.source]


class World:
    def __init__(self, scene: Scene, clock: Callable[[], float] = time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._scene = scene
        now = clock()
        self._body_provenance: Dict[str, Provenance] = {name: Provenance(INITIAL, now) for name in scene.bodies}
        self._robot: Optional[JointConfiguration] = None
        self._robot_provenance: Optional[Provenance] = None
        self._gripper: Optional[GripperState] = None
        self._held_body: Optional[str] = None
        self._gripper_provenance: Optional[Provenance] = None
        self._listeners: List[Callable[[WorldChange], None]] = []

    # --- Lectura -------------------------------------------------------------------

    @property
    def scene(self) -> Scene:
        return self._scene

    def body_provenance(self, name: str) -> Provenance:
        return self._body_provenance[name]

    @property
    def robot_configuration(self) -> Optional[JointConfiguration]:
        return self._robot

    @property
    def gripper(self) -> Optional[GripperState]:
        return self._gripper

    @property
    def holding(self) -> Optional[bool]:
        """None si no se sabe (sin pinza o sin leerla todavía)."""
        return None if self._gripper is None else self._gripper.holding_object

    @property
    def held_body(self) -> Optional[str]:
        """Qué cuerpo se sujeta, si se sabe: la pinza real solo dice QUE
        sujeta algo; el nombre lo pone quien ejecuta el `pick`."""
        return self._held_body

    # --- Actualización ---------------------------------------------------------

    def subscribe(self, listener: Callable[[WorldChange], None]) -> None:
        self._listeners.append(listener)

    def move_body(self, name: str, pose: Pose, source: str, timestamp: Optional[float] = None) -> bool:
        """Nueva pose de un cuerpo conocido. Devuelve si se aplicó."""
        new = Provenance(source, self._clock() if timestamp is None else timestamp)
        with self._lock:
            if name not in self._scene.bodies:
                raise KeyError(f'no hay ningún cuerpo "{name}" en el mundo')
            if not accepts(self._body_provenance[name], new):
                return False
            self._scene = self._scene.with_body(name, replace(self._scene.bodies[name], pose=pose))
            self._body_provenance[name] = new
        self._notify(WorldChange("body", source, name))
        return True

    def observe_body(self, name: str, body: Body, source: str, timestamp: Optional[float] = None) -> bool:
        """Un cuerpo visto entero (forma incluida), nuevo o ya conocido --
        lo que aporta la percepción."""
        new = Provenance(source, self._clock() if timestamp is None else timestamp)
        with self._lock:
            current = self._body_provenance.get(name)
            if current is not None and not accepts(current, new):
                return False
            self._scene = self._scene.with_body(name, body)
            self._body_provenance[name] = new
        self._notify(WorldChange("body", source, name))
        return True

    def set_robot(self, configuration: JointConfiguration, source: str) -> None:
        with self._lock:
            self._robot = configuration
            self._robot_provenance = Provenance(source, self._clock())
        self._notify(WorldChange("robot", source))

    def set_gripper(self, state: GripperState, source: str, held_body: Optional[str] = None) -> None:
        """Estado de la pinza. `held_body` es el nombre de lo sujeto, si se
        sabe; si la pinza ya no sujeta nada, se olvida."""
        with self._lock:
            self._gripper = state
            if not state.holding_object:
                self._held_body = None
            elif held_body is not None:
                self._held_body = held_body
            self._gripper_provenance = Provenance(source, self._clock())
        self._notify(WorldChange("gripper", source))

    def _notify(self, change: WorldChange) -> None:
        for listener in list(self._listeners):
            listener(change)
