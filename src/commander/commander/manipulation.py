"""Herramientas de manipulación: coger y dejar cuerpos de la `Scene` con un
brazo y una pinza, escritas SOLO contra los puertos del dominio
(`RobotConnectorPort`, `GripperPort`, `KinematicsPort`). No saben si debajo
hay CoppeliaSim o el CR5 real: eso lo decide quien las monta (ver
`cell/direct.py`). Un script de prueba se reduce a describir la célula
(`descriptions/cells/*.yaml`) y pedir `pick(...)`/`place(...)`.

Lo que cambia entre simulación y robot real no está aquí, se inyecta:
- `wait_until_idle`: el CR5 real ENCOLA los MovJ; hay que esperar a que
  termine antes de tocar la pinza o de leer dónde está. En simulación
  `set_joints` ya es instantáneo.
- `confirm`: antes de cada bajada hacia la mesa se pide confirmación. En
  el real, por teclado; en simulación, siempre sí.
- `step_pause_seconds`: pausa entre waypoints (para ver la animación, o
  para no saturar la cola del controlador).

Convención de herramienta: el eje de la herramienta es el z de la brida
(`forward_kinematics`), y lo que la pinza agarra queda centrado en ese
eje, a `GraspSettings.grasp_offset` de la brida. Coger un cuerpo es llevar
la brida a `centro - grasp_offset·eje`, sin cambiar la orientación actual.
Por eso, antes de `pick`, el brazo tiene que estar ya en una postura con
la herramienta en la orientación en que se quiere coger (p. ej. mirando
hacia abajo): no hay todavía planificador de agarres que la elija.

Los tramos cerca de la mesa (aproximación, bajada, subida) son rectas
CARTESIANAS de verdad (`move_linear`): `KinematicsPort.compute_trajectory`
interpola en espacio articular hacia la solución de la IK, así que el
camino entre medias no es recto aunque el punto final lo sea. Los
desplazamientos libres, ya en altura, sí usan esa interpolación
(`move_to_pose`).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

from shared_kernel import (
    Body,
    GripperPort,
    GripperState,
    JointConfiguration,
    KinematicsPort,
    Point,
    Pose,
    RobotConnectorPort,
    Trajectory,
)

Vector = Tuple[float, float, float]


class GraspFailedError(Exception):
    """La pinza cerró sin coger nada (`holding_object` es False)."""


class OperationCancelledError(Exception):
    """`confirm` dijo que no: se para antes de moverse."""


class NoGripperError(Exception):
    """Se pidió abrir, cerrar, coger o dejar en una célula sin pinza."""


@dataclass(frozen=True)
class GraspSettings:
    """Parámetros de un agarre. `grasp_offset` depende de CÓMO está montada
    la pinza (acoplador incluido): en simulación sale del modelo
    (`grasp.offset` en `descriptions/tools/`); en el robot real hay que
    MEDIRLO.

    - `approach_distance`: a cuánto se para antes de bajar a coger o a
      dejar, medido a lo largo del eje de la herramienta.
    - `lift_distance`: cuánto sube con el cuerpo antes de llevarlo.
    - `linear_step`: separación entre los objetivos intermedios de una
      recta cartesiana.
    - `gripper_timeout_seconds` / `gripper_poll_seconds`: espera a la
      pinza tras cada orden (la real vuelve en cuanto la acepta)."""

    grasp_offset: float
    approach_distance: float = 0.10
    lift_distance: float = 0.15
    linear_step: float = 0.005
    gripper_timeout_seconds: float = 6.0
    gripper_poll_seconds: float = 0.2


def tool_axis(pose: Pose) -> Vector:
    """Eje z de la brida en el marco base: tercera columna de la matriz de
    rotación del cuaternión de `pose`."""
    x, y, z, w = pose.qx, pose.qy, pose.qz, pose.qw
    return (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))


def _offset(pose: Pose, axis: Vector, distance: float) -> Pose:
    """`pose` desplazada `distance` a lo largo de `axis`, misma orientación."""
    return Pose(
        pose.x + distance * axis[0],
        pose.y + distance * axis[1],
        pose.z + distance * axis[2],
        pose.qx,
        pose.qy,
        pose.qz,
        pose.qw,
    )


class Manipulator:
    def __init__(
        self,
        robot: RobotConnectorPort,
        gripper: Optional[GripperPort],
        kinematics: KinematicsPort,
        settings: GraspSettings,
        wait_until_idle: Callable[[], None] = lambda: None,
        confirm: Callable[[str], bool] = lambda message: True,
        step_pause_seconds: float = 0.0,
        log: Callable[[str], None] = print,
    ):
        self.robot = robot
        self.gripper = gripper
        self.kinematics = kinematics
        self.settings = settings
        self._wait_until_idle = wait_until_idle
        self._confirm = confirm
        self._step_pause_seconds = step_pause_seconds
        self._log = log

    # --- Brazo ---------------------------------------------------------------

    def current_configuration(self) -> JointConfiguration:
        return self.robot.get_current_configuration()

    def flange_pose(self) -> Pose:
        return self.kinematics.forward_kinematics(self.current_configuration())

    def move_joints(self, target: JointConfiguration, steps: int = 50) -> None:
        """Movimiento libre en espacio articular (p. ej. a una postura de
        trabajo)."""
        self._play(Trajectory.straight_line(self.current_configuration(), target, steps).waypoints)

    def move_to_pose(self, goal: Pose) -> None:
        """Movimiento libre a una pose de la brida: IK + interpolación
        articular. El camino NO es recto; úsalo lejos de la mesa."""
        trajectory = self.kinematics.compute_trajectory(goal, self.current_configuration())
        self._play(trajectory.waypoints)

    def move_linear(self, goal: Pose) -> None:
        """Recta cartesiana de la brida hasta la POSICIÓN de `goal`, con la
        orientación de `goal` desde el primer paso: un objetivo cada
        `linear_step` metros, cada uno resuelto con IK desde el anterior."""
        start = self.flange_pose()
        length = math.dist((start.x, start.y, start.z), (goal.x, goal.y, goal.z))
        steps = max(1, math.ceil(length / self.settings.linear_step))
        configuration = self.current_configuration()
        waypoints: List[JointConfiguration] = []
        for i in range(1, steps + 1):
            t = i / steps
            target = Pose(
                start.x + (goal.x - start.x) * t,
                start.y + (goal.y - start.y) * t,
                start.z + (goal.z - start.z) * t,
                goal.qx,
                goal.qy,
                goal.qz,
                goal.qw,
            )
            configuration = self.kinematics.compute_trajectory(target, configuration).waypoints[-1]
            waypoints.append(configuration)
        self._play(waypoints)

    def _play(self, waypoints: Sequence[JointConfiguration]) -> None:
        for waypoint in waypoints:
            self.robot.set_joints(waypoint)
            if self._step_pause_seconds:
                time.sleep(self._step_pause_seconds)
        self._wait_until_idle()

    # --- Pinza ---------------------------------------------------------------

    def open(self) -> GripperState:
        return self._move_gripper(0.0)

    def close(self) -> GripperState:
        return self._move_gripper(1.0)

    def _move_gripper(self, opening: float) -> GripperState:
        """`set_opening` vuelve en cuanto la orden se acepta (ver
        GripperPort): se espera a que la pinza detecte un objeto o deje de
        moverse, para que la secuencia sea de verdad secuencial."""
        if self.gripper is None:
            raise NoGripperError("esta célula no tiene pinza montada")
        self.gripper.set_opening(opening)
        deadline = time.monotonic() + self.settings.gripper_timeout_seconds
        state = self.gripper.get_state()
        previous: Optional[float] = None
        stable_reads = 0
        while time.monotonic() < deadline:
            if state.holding_object and opening > 0.0:
                return state
            if previous is not None and abs(state.opening - previous) < 0.005:
                stable_reads += 1
                if stable_reads >= 2:
                    return state
            else:
                stable_reads = 0
            previous = state.opening
            time.sleep(self.settings.gripper_poll_seconds)
            state = self.gripper.get_state()
        self._log(f"Aviso: la pinza no se estabilizó en {self.settings.gripper_timeout_seconds}s.")
        return state

    # --- Coger y dejar -------------------------------------------------------

    def grasp_pose_for(self, center: Point) -> Pose:
        """Pose de la brida para que `center` quede entre las yemas, con la
        orientación actual de la herramienta."""
        current = self.flange_pose()
        at_center = Pose(center.x, center.y, center.z, current.qx, current.qy, current.qz, current.qw)
        return _offset(at_center, tool_axis(current), -self.settings.grasp_offset)

    def move_above(self, center: Point) -> None:
        """Se coloca encima de `center`, a `approach_distance` del punto de
        agarre a lo largo del eje de la herramienta, sin cambiar su
        orientación: el mismo sitio desde el que `pick`/`place` bajan."""
        grasp = self.grasp_pose_for(center)
        self.move_to_pose(_offset(grasp, tool_axis(grasp), -self.settings.approach_distance))

    def pick(self, name: str, body: Body) -> GripperState:
        """Coge `body`: se coloca sobre él (a `approach_distance`), baja en
        recta, cierra, comprueba `holding_object` y sube `lift_distance`.
        Si no coge nada, abre, se retira y lanza `GraspFailedError`."""
        if self.gripper is None:
            raise NoGripperError("esta célula no tiene pinza montada")
        if not body.graspable:
            raise ValueError(f'"{name}" no es un cuerpo que se pueda coger (graspable=False)')
        settings = self.settings
        grasp = self.grasp_pose_for(Point(body.pose.x, body.pose.y, body.pose.z))
        axis = tool_axis(grasp)
        above = _offset(grasp, axis, -settings.approach_distance)

        self._log(f'Coger "{name}": abrir y colocarse encima...')
        self.open()
        self.move_to_pose(above)
        self._ask(f'Bajar {settings.approach_distance * 100:.0f} cm para coger "{name}"')
        self.move_linear(grasp)

        state = self.close()
        self._log(f"  pinza: opening={state.opening:.2f} sujeta={'SÍ' if state.holding_object else 'no'}")
        if not state.holding_object:
            self.open()
            self.move_linear(above)
            raise GraspFailedError(f'la pinza cerró sin coger "{name}"')

        self.move_linear(_offset(grasp, axis, -settings.lift_distance))
        return state

    def place(self, center: Point) -> GripperState:
        """Deja lo que sujeta con su centro en `center`: se desplaza a
        `lift_distance` por encima, baja en recta, abre y se retira
        `approach_distance`."""
        settings = self.settings
        release = self.grasp_pose_for(center)
        axis = tool_axis(release)

        self._log(f"Dejar en ({center.x:+.3f}, {center.y:+.3f}, {center.z:+.3f})...")
        self.move_to_pose(_offset(release, axis, -settings.lift_distance))
        self._ask(f"Bajar {settings.lift_distance * 100:.0f} cm para dejarlo")
        self.move_linear(release)
        state = self.open()
        self.move_linear(_offset(release, axis, -settings.approach_distance))
        return state

    def _ask(self, message: str) -> None:
        if not self._confirm(message):
            raise OperationCancelledError(message)
