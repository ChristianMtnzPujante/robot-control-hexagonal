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
eje, a `GraspSettings.grasp_offset` de la brida. Coger, dejar, ponerse
encima o ir a una posición es siempre DESDE ARRIBA, con la herramienta
mirando hacia abajo (`top_down_quaternion`: si ya mira hacia abajo,
conserva su giro). Hasta el 01/10 se conservaba la orientación actual, y
desde la home del CR5 (herramienta horizontal) "encima" salía de lado.
Otras direcciones de agarre necesitan un planificador de agarres.

Si la IK no converge desde la postura actual, `move_to_pose` reintenta
desde las posturas conocidas (`ik_seeds`): PoE resuelve por iteración
local, y desde el borde del alcance puede no llegar.

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
from dataclasses import dataclass, replace
from typing import Callable, List, Optional, Sequence, Tuple

from shared_kernel import (
    Body,
    GripperPort,
    GripperState,
    JointConfiguration,
    JointPosition,
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


def _rotation_columns(pose: Pose) -> Tuple[Vector, Vector, Vector]:
    """Ejes x, y, z de la orientación de `pose`, en el marco base."""
    x, y, z, w = pose.qx, pose.qy, pose.qz, pose.qw
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),
        (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)),
        tool_axis(pose),
    )


def _quaternion_from_axes(x_axis: Vector, y_axis: Vector, z_axis: Vector) -> Tuple[float, float, float, float]:
    """Cuaternión (qx, qy, qz, qw) de la rotación cuyas columnas son esos
    ejes (método de Shepperd, estable en todos los casos)."""
    m = ((x_axis[0], y_axis[0], z_axis[0]), (x_axis[1], y_axis[1], z_axis[1]), (x_axis[2], y_axis[2], z_axis[2]))
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4)
    i = max(range(3), key=lambda k: m[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + m[i][i] - m[j][j] - m[k][k])
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (m[j][i] + m[i][j]) / s
    q[k] = (m[k][i] + m[i][k]) / s
    return (q[0], q[1], q[2], (m[k][j] - m[j][k]) / s)


_DOWN: Vector = (0.0, 0.0, -1.0)
_ALREADY_DOWN_COS = math.cos(math.radians(10))


def top_down_quaternion(current: Pose) -> Tuple[float, float, float, float]:
    """Orientación con la herramienta mirando hacia ABAJO (z de la brida =
    -z del mundo). Si `current` ya mira hacia abajo (menos de 10°), se
    conserva tal cual, giro incluido. Si no, el giro alrededor de la
    vertical sale de proyectar el x actual de la brida en el plano
    horizontal (o el y del mundo si es casi vertical)."""
    x_axis, _, z_axis = _rotation_columns(current)
    if sum(a * b for a, b in zip(z_axis, _DOWN)) >= _ALREADY_DOWN_COS:
        return (current.qx, current.qy, current.qz, current.qw)
    horizontal = (x_axis[0], x_axis[1], 0.0)
    norm = math.hypot(horizontal[0], horizontal[1])
    x_new = (horizontal[0] / norm, horizontal[1] / norm, 0.0) if norm > 0.1 else (0.0, 1.0, 0.0)
    # y = z × x, con z = abajo
    y_new = (_DOWN[1] * x_new[2] - _DOWN[2] * x_new[1],
             _DOWN[2] * x_new[0] - _DOWN[0] * x_new[2],
             _DOWN[0] * x_new[1] - _DOWN[1] * x_new[0])
    return _quaternion_from_axes(x_new, y_new, _DOWN)


def _nearest_turn(solution: JointConfiguration, reference: JointConfiguration) -> JointConfiguration:
    """La misma configuración, con cada articulación en la vuelta (±360°)
    más cercana a `reference`: misma pose, sin girar de más."""
    positions = []
    for position in solution.positions:
        angle, ref = position.angle_radians, reference.angle_of(position.joint_name)
        positions.append(JointPosition(position.joint_name, ref + math.remainder(angle - ref, 2 * math.pi)))
    return JointConfiguration.create(positions).value


def _max_travel_deg(a: JointConfiguration, b: JointConfiguration) -> float:
    return max(abs(math.degrees(b.angle_of(p.joint_name) - p.angle_radians)) for p in a.positions)


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


class RecordingRobot:
    """`RobotConnectorPort` que no mueve nada: guarda cada waypoint. Base del
    ensayo en seco (`Manipulator.dry_run`)."""

    def __init__(self, configuration: JointConfiguration):
        self.configuration = configuration
        self.waypoints: List[JointConfiguration] = []

    def set_joints(self, configuration: JointConfiguration) -> None:
        self.configuration = configuration
        self.waypoints.append(configuration)

    def get_current_configuration(self) -> JointConfiguration:
        return self.configuration

    def close(self) -> None:
        pass


class _DryRunGripper:
    """`GripperPort` de ensayo: al cerrar dice que sujeta algo, para poder
    planificar un `pick` completo sin pinza."""

    def __init__(self):
        self._state = GripperState(0.0, True, False, 0)

    def activate(self) -> None:
        pass

    def set_opening(self, fraction: float) -> None:
        self._state = GripperState(fraction, True, fraction > 0.0, 0)

    def get_state(self) -> GripperState:
        return self._state

    def close(self) -> None:
        pass


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
        ik_seeds: Sequence[JointConfiguration] = (),
    ):
        """`ik_seeds`: configuraciones desde las que reintentar la IK si no
        converge desde la actual (p. ej. las posturas con nombre de la
        célula): PoE resuelve por iteración local y, desde el borde del
        alcance (la home del CR5), puede no llegar a objetivos lejanos."""
        self.robot = robot
        self._ik_seeds = tuple(ik_seeds)
        self.gripper = gripper
        self.kinematics = kinematics
        self.settings = settings
        self._wait_until_idle = wait_until_idle
        self._confirm = confirm
        self._step_pause_seconds = step_pause_seconds
        self._log = log

    @property
    def ik_seeds(self) -> Tuple[JointConfiguration, ...]:
        return self._ik_seeds

    def dry_run(self) -> Tuple["Manipulator", RecordingRobot]:
        """Un gemelo de este `Manipulator` con la misma cinemática, ajustes y
        semillas, pero sobre un robot que solo GRABA, partiendo de la
        configuración actual. Llamar a una operación sobre el gemelo da los
        waypoints exactos que mandaría la real (todo es determinista), sin
        mover nada: es lo que se revisa antes de ejecutar (ver
        `motion_check.py`)."""
        recorder = RecordingRobot(self.current_configuration())
        twin = Manipulator(
            recorder,
            _DryRunGripper() if self.gripper is not None else None,
            self.kinematics,
            replace(self.settings, gripper_poll_seconds=0.0, gripper_timeout_seconds=0.0),
            ik_seeds=self._ik_seeds,
            log=lambda message: None,
        )
        return twin, recorder

    # --- Brazo ---------------------------------------------------------------

    def current_configuration(self) -> JointConfiguration:
        return self.robot.get_current_configuration()

    def flange_pose(self) -> Pose:
        return self.kinematics.forward_kinematics(self.current_configuration())

    def move_joints(self, target: JointConfiguration, steps: int = 50) -> None:
        """Movimiento libre en espacio articular (p. ej. a una postura de
        trabajo)."""
        self._play(Trajectory.straight_line(self.current_configuration(), target, steps).waypoints)

    def move_to_pose(self, goal: Pose, max_step_deg: float = 2.0) -> None:
        """Movimiento libre a una pose de la brida: IK + interpolación
        articular. El camino NO es recto; úsalo lejos de la mesa.

        La IK de PoE es local y puede converger a una solución lejana (con
        vueltas de más o en otra rama): desde la home del CR5, un `pick`
        llegó a pedir joint2 = +309°, y la interpolación habría metido la
        herramienta por debajo de la mesa (cazado en seco por
        `real_cell_check`, 01/10). Por eso: se resuelve desde la postura
        actual Y desde cada semilla (`ik_seeds`), cada solución se lleva a
        la vuelta equivalente más cercana a la actual en cada articulación,
        y se elige la que MENOS mueve. Se interpola en pasos de como mucho
        `max_step_deg`."""
        current = self.current_configuration()
        candidates = []
        failure: Optional[Exception] = None
        for seed in (current, *self._ik_seeds):
            try:
                solution = self.kinematics.compute_trajectory(goal, seed).waypoints[-1]
            except RuntimeError as error:
                failure = failure or error
                continue
            candidates.append(_nearest_turn(solution, current))
        if not candidates:
            raise RuntimeError(
                f"no hay solución de IK para ({goal.x:+.3f}, {goal.y:+.3f}, {goal.z:+.3f}) ni desde la postura "
                f"actual ni desde {len(self._ik_seeds)} postura(s) conocida(s): probablemente fuera de alcance. "
                f"({failure})"
            )
        target = min(candidates, key=lambda c: _max_travel_deg(current, c))
        steps = max(1, math.ceil(_max_travel_deg(current, target) / max_step_deg))
        self._play(Trajectory.straight_line(current, target, steps).waypoints)

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
        herramienta mirando hacia ABAJO (ver `top_down_quaternion`): coger y
        dejar son siempre desde arriba, hasta que haya un planificador de
        agarres. Sin herramienta (`grasp_offset` 0), es la brida en `center`."""
        qx, qy, qz, qw = top_down_quaternion(self.flange_pose())
        at_center = Pose(center.x, center.y, center.z, qx, qy, qz, qw)
        return _offset(at_center, _DOWN, -self.settings.grasp_offset)

    def move_to_position(self, center: Point) -> None:
        """Lleva el punto de agarre (entre las yemas; sin herramienta, la
        brida) a `center`, con la herramienta hacia abajo. Movimiento libre:
        el camino no es recto."""
        self.move_to_pose(self.grasp_pose_for(center))

    def move_above(self, center: Point) -> None:
        """Se coloca encima de `center`, a `approach_distance` del punto de
        agarre en vertical, con la herramienta hacia abajo: el mismo sitio
        desde el que `pick`/`place` bajan."""
        grasp = self.grasp_pose_for(center)
        self.move_to_pose(_offset(grasp, _DOWN, -self.settings.approach_distance))

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
