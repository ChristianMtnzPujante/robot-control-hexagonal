"""Tercer implementador mínimo de `PlanningPort` (shared_kernel), esta vez
NO de evitación de obstáculos EXTERNOS (eso son
`obstacle_avoiding_planning_adapter.py`/`whole_body_obstacle_avoiding_planning_adapter.py`)
sino de AUTOcolisión -- que ningún eslabón del robot choque con otro
eslabón del propio robot, haya o no obstáculos externos de por medio.
Motivado por un hallazgo real (08/09): `GetErrorID()`=[76] del CR5 físico
durante `cr5_semicircle_demo.py` ("el extremo interfiere con el cuerpo del
robot") -- el propio driver del fabricante ya detiene el robot cuando pasa,
pero para entonces ya se mandó un `MovJ` real. Este adaptador comprueba
ANTES de mandar nada.

Modelo geométrico: cada eslabón se aproxima como una CÁPSULA -- el
segmento entre dos poses consecutivas de `KinematicsPort.link_poses`
(igual que `_body_segments` en `whole_body_obstacle_avoiding_planning_adapter.py`)
más un radio fijo, `link_radius_meters`. Dos eslabones colisionan si la
distancia entre sus segmentos (`_segment_geometry.segment_segment_distance`)
es menor que la suma de sus radios -- aquí, el doble del mismo radio
único, ver más abajo.

Radio ÚNICO, calibrado contra DOS puntos reales (08/09), no medido de la
malla ni adivinado a ciegas: el URDF del CR5 solo trae mallas como
geometría de colisión (`Link1.dae`...`Link6.dae`, sin primitivas con
radio), así que no hay un número "real" que leer de ningún sitio sin
procesar las mallas a mano -- pero sí hay dos referencias reales con las
que contrastar cualquier valor antes de fijarlo:

  - En la home (segura por definición -- el robot descansa ahí sin
    problema), la distancia real más corta entre eslabones NO adyacentes
    (`joint3->joint4` vs `joint5->joint6`, el par más cercano de los seis
    posibles) es 11.6cm.
  - En la secuencia de 10 waypoints que SÍ disparó la alarma real del
    fabricante (`cr5_semicircle_demo.py`, GetErrorID()=[76]), esa misma
    distancia bajó progresivamente hasta 7.7cm en el último tramo
    calculado (joint4≈-143°, joint6≈89°) -- justo la zona del recorrido
    donde se produjo la autocolisión real.

`_DEFAULT_LINK_RADIUS_METERS = 0.04` da un requisito de separación de
2×0.04 = 8cm: dentro del hueco entre ambas referencias (deja 3.6cm de
margen sobre la home, y ya habría rechazado la trayectoria real ANTES del
último tramo, no solo en el punto exacto del incidente). Sigue siendo una
aproximación de UN radio para los 6 eslabones (el CR5 no es uniforme de
grosor de verdad) -- ajustable por constructor si en el futuro se mide con
precisión por eslabón (radios distintos, no todos iguales), o si otro
robot con proporciones distintas necesita recalibrar estos dos puntos de
referencia con sus propios datos.

Pares estructuralmente conectados solamente se excluyen -- no basta con
excluir "el índice siguiente" (i, i+1): el CR5 tiene `joint1`/`joint2` en
el MISMO punto físico (offset cero entre ellos en el URDF -- dos ejes que
se cruzan en el "hombro"), así que el eslabón base->joint1 y el eslabón
joint2->joint3 comparten un extremo aunque sus índices no sean
consecutivos -- encontrado en vivo (08/09): con una exclusión ingenua por
índice, ese par salía como "en colisión" en TODAS las configuraciones,
incluida la propia home, un falso positivo permanente. La exclusión real
(`_structural_adjacency_exclusions`) es GEOMÉTRICA: dos segmentos se
excluyen si comparten un extremo (a menos de `_STRUCTURAL_TOLERANCE_METERS`)
en una configuración de referencia CUALQUIERA -- es una propiedad del
robot, no de la postura, así que basta calcularlo una vez y cachearlo.

`SelfCollisionAwarePlanningAdapter` NO intenta rodear la autocolisión (no
hay, todavía, un algoritmo de búsqueda tipo RRT/CHOMP que sepa moverse en
espacio de articulaciones para esquivarla en general -- ver ROADMAP.md,
Bloque 4): si algún waypoint de la trayectoria calculada colisiona consigo
mismo, se rechaza la trayectoria ENTERA con un error explícito, mismo
criterio que `PoeKinematicsAdapter` (IK que no converge) y
`_within_a_full_turn` (ángulo fuera de ±2π) -- fallar alto y explícito, no
mandar algo peligroso o incorrecto.

`SelfCollisionAvoidingPlanningAdapter` (08/09) SÍ intenta esquivarla, pero
con un truco concreto de este tipo de brazo (muñeca esférica de 3 ejes),
no una búsqueda general: cerca de `joint5≈0` (la misma singularidad ya
documentada en `PoeKinematicsAdapter` -- ejes de joint4/joint6 casi
paralelos ahí) hay un grado de libertad casi redundante entre esos dos
joints para una orientación dada -- moverlos en direcciones opuestas
apenas cambia la orientación final, pero SÍ desplaza la posición del
tramo intermedio (el brazo joint4->joint5 no tiene longitud cero), porque
`joint4` es la articulación que sostiene a `joint5`/`joint6`. Ese
desplazamiento es justo la palanca que hace falta: reintentar la IK del
MISMO objetivo cartesiano partiendo de una semilla con `joint4`/`joint6`
ligeramente adelantados en direcciones opuestas empuja a Newton-Raphson
hacia una rama de solución distinta -- misma pose exacta al converger,
pero con el tramo `joint4->joint6` desplazado lo suficiente para dejar de
solaparse con el resto del brazo. Verificado en vivo (08/09) contra la
secuencia real que disparó la alarma física: un desplazamiento de
apenas 4° ya basta para el punto que colisionaba, con un salto adicional
de solo ~10° respecto al waypoint anterior (nada que ver con las
alternativas de "vuelta de muñeca" completa, ~180°, mucho más bruscas).
"""

from __future__ import annotations

import math
from typing import FrozenSet, List, Optional, Tuple

import numpy as np

from shared_kernel import JointConfiguration, JointPosition, KinematicsPort, Pose, Scene, Trajectory

from ._segment_geometry import segment_segment_distance

# Ver docstring del módulo -- calibrado contra home (segura, 11.6cm) y la
# secuencia real que disparó la alarma (7.7cm en el peor tramo).
_DEFAULT_LINK_RADIUS_METERS = 0.04

# Por debajo de esto, dos extremos de segmento se consideran "el mismo
# punto físico" (mismo eje, offset ~0 en el URDF) -- no una distancia de
# colisión real, es tolerancia numérica/de fabricación.
_STRUCTURAL_TOLERANCE_METERS = 1e-3


class SelfCollisionError(RuntimeError):
    """Alguna configuración de la trayectoria calculada haría que dos
    eslabones no conectados del propio robot se solaparan -- ver
    `find_self_collision`."""


def _body_segments(link_poses: List[Pose]) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Mismo cálculo que `_body_segments` en
    `whole_body_obstacle_avoiding_planning_adapter.py` (duplicado a
    propósito -- ver ese módulo: es la única función que ambos comparten
    conceptualmente, y no merece una extracción por sí sola)."""
    points = [np.zeros(3)] + [np.array([p.x, p.y, p.z]) for p in link_poses]
    return list(zip(points[:-1], points[1:]))


def structural_adjacency_exclusions(
    reference_link_poses: List[Pose],
    tolerance_meters: float = _STRUCTURAL_TOLERANCE_METERS,
) -> FrozenSet[Tuple[int, int]]:
    """Pares de eslabones (por índice de segmento) que están conectados
    FÍSICAMENTE -- comparten un extremo en `reference_link_poses` --, y por
    tanto se excluyen SIEMPRE de `find_self_collision`, en cualquier
    configuración. Es una propiedad de la geometría del robot
    (`RobotDescription`), no de la postura concreta: dos ejes que se cruzan
    en el mismo punto (offset cero entre ellos en el URDF, como
    joint1/joint2 del CR5) siguen compartiendo ese punto se gire lo que se
    gire -- por eso basta una única `reference_link_poses` (CUALQUIERA,
    p. ej. la primera configuración que se le pase al adaptador) para
    detectarlos todos, no hace falta recalcular por waypoint.

    Cubre tanto los pares "consecutivos de toda la vida" (segmento i y
    i+1 comparten la articulación entre ambos) como el caso menos obvio
    encontrado en vivo el 08/09: segmentos que NO son consecutivos por
    índice pero comparten extremo porque el eslabón intermedio tiene
    longitud ~0 (ver docstring del módulo)."""
    segments = _body_segments(reference_link_poses)
    excluded = set()
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            closest_gap = min(
                float(np.linalg.norm(endpoint_i - endpoint_j))
                for endpoint_i in segments[i]
                for endpoint_j in segments[j]
            )
            if closest_gap < tolerance_meters:
                excluded.add((i, j))
    return frozenset(excluded)


def find_self_collision(
    link_poses: List[Pose],
    link_radius_meters: float,
    excluded_pairs: FrozenSet[Tuple[int, int]] = frozenset(),
) -> Optional[Tuple[int, int, float]]:
    """Recorre todos los pares de eslabones que NO estén en
    `excluded_pairs` (ver `structural_adjacency_exclusions`) y devuelve
    `(índice_a, índice_b, penetración)` del peor solapamiento encontrado
    (mayor penetración), o `None` si ninguno colisiona. Los índices son
    posiciones en `_body_segments` (0 = base->primera articulación, ...,
    n-1 = última articulación->tip)."""
    segments = _body_segments(link_poses)
    worst: Optional[Tuple[int, int, float]] = None
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            if (i, j) in excluded_pairs:
                continue
            distance = segment_segment_distance(*segments[i], *segments[j])
            penetration = 2 * link_radius_meters - distance
            if penetration <= 0:
                continue  # este par tiene margen de sobra, no es el peor
            if worst is None or penetration > worst[2]:
                worst = (i, j, penetration)
    return worst


def _first_trajectory_collision(
    trajectory: Trajectory,
    kinematics: KinematicsPort,
    link_radius_meters: float,
    excluded_pairs: FrozenSet[Tuple[int, int]],
) -> Optional[Tuple[int, int, int, float]]:
    """Recorre los waypoints de `trajectory` EN ORDEN y devuelve
    `(índice_waypoint, eslabón_a, eslabón_b, penetración)` del primero que
    colisiona, o `None` si ninguno lo hace. Compartido por las dos clases
    de este módulo."""
    for index, waypoint in enumerate(trajectory.waypoints):
        collision = find_self_collision(
            kinematics.link_poses(waypoint), link_radius_meters, excluded_pairs
        )
        if collision is not None:
            link_a, link_b, penetration = collision
            return index, link_a, link_b, penetration
    return None


class SelfCollisionAwarePlanningAdapter:
    def __init__(
        self,
        kinematics: KinematicsPort,
        link_radius_meters: float = _DEFAULT_LINK_RADIUS_METERS,
    ) -> None:
        self._kinematics = kinematics
        self._link_radius_meters = link_radius_meters
        # Se calcula en la primera llamada (necesita una configuración real
        # para derivar link_poses) y se cachea -- es una propiedad de la
        # geometría del robot, no de la postura, así que no cambia entre
        # llamadas (ver structural_adjacency_exclusions).
        self._structural_exclusions: Optional[FrozenSet[Tuple[int, int]]] = None

    def compute_trajectory(
        self,
        goal: Pose,
        current_configuration: JointConfiguration,
        scene: Scene,
    ) -> Trajectory:
        # scene (obstáculos externos) se ignora a propósito -- este
        # planificador resuelve un único problema, autocolisión, igual que
        # NaivePlanningAdapter ignora la Scene por completo para el suyo
        # (ningún chequeo). Combinarlo con evitación de obstáculos externos
        # es responsabilidad de quien componga varios PlanningPort, no de
        # este adaptador.
        del scene
        if self._structural_exclusions is None:
            self._structural_exclusions = structural_adjacency_exclusions(
                self._kinematics.link_poses(current_configuration)
            )
        trajectory = self._kinematics.compute_trajectory(goal, current_configuration)
        collision = _first_trajectory_collision(
            trajectory, self._kinematics, self._link_radius_meters, self._structural_exclusions
        )
        if collision is None:
            return trajectory
        index, link_a, link_b, penetration = collision
        raise SelfCollisionError(
            f"Waypoint {index}/{len(trajectory.waypoints) - 1}: autocolisión "
            f"entre el eslabón {link_a} y el eslabón {link_b} "
            f"(penetración {penetration * 100:.1f}cm con un radio de cápsula "
            f"de {self._link_radius_meters * 100:.0f}cm por eslabón) -- "
            "trayectoria rechazada, no se ha mandado nada al robot."
        )


class SelfCollisionAvoidingPlanningAdapter:
    """Como `SelfCollisionAwarePlanningAdapter`, pero antes de rendirse
    intenta esquivar la autocolisión reintentando la IK del mismo objetivo
    desde una semilla con los dos joints exteriores de la muñeca
    (`positions[-3]`/`positions[-1]` -- ver docstring del módulo sobre por
    qué esos dos y no otros) desplazados en direcciones opuestas, en pasos
    crecientes. Solo tiene sentido para brazos con muñeca esférica de al
    menos 3 joints -- con menos, se comporta exactamente como
    `SelfCollisionAwarePlanningAdapter` (nunca hay nada que desplazar).

    Limitación real, no resuelta: reconstruye la trayectoria final como
    `[current_configuration, solución]`, DOS waypoints -- si el
    `KinematicsPort` envuelto interpola en varios pasos (`steps>1`), esa
    interpolación se descarta. Pensado para el uso real de hoy
    (`PoeKinematicsAdapter(steps=1)`, un punto cartesiano por llamada, ver
    `cr5_semicircle_sim_demo.py`/`cr5_semicircle_demo.py`), no para
    trayectorias multi-paso de una sola llamada."""

    def __init__(
        self,
        kinematics: KinematicsPort,
        link_radius_meters: float = _DEFAULT_LINK_RADIUS_METERS,
        nudge_step_degrees: float = 2.0,
        max_nudge_degrees: float = 90.0,
    ) -> None:
        self._kinematics = kinematics
        self._link_radius_meters = link_radius_meters
        self._nudge_step_degrees = nudge_step_degrees
        self._max_nudge_degrees = max_nudge_degrees
        self._structural_exclusions: Optional[FrozenSet[Tuple[int, int]]] = None

    def compute_trajectory(
        self,
        goal: Pose,
        current_configuration: JointConfiguration,
        scene: Scene,
    ) -> Trajectory:
        del scene  # ver SelfCollisionAwarePlanningAdapter -- mismo motivo
        if self._structural_exclusions is None:
            self._structural_exclusions = structural_adjacency_exclusions(
                self._kinematics.link_poses(current_configuration)
            )
        trajectory = self._kinematics.compute_trajectory(goal, current_configuration)
        collision = _first_trajectory_collision(
            trajectory, self._kinematics, self._link_radius_meters, self._structural_exclusions
        )
        if collision is None:
            return trajectory

        wrist_pair = self._wrist_joint_pair(current_configuration)
        if wrist_pair is not None:
            outer_joint, opposite_joint = wrist_pair
            nudge_degrees = self._nudge_step_degrees
            while nudge_degrees <= self._max_nudge_degrees:
                for sign in (1.0, -1.0):
                    seed = _nudge_configuration(
                        current_configuration, outer_joint, opposite_joint, sign * nudge_degrees
                    )
                    try:
                        candidate = self._kinematics.compute_trajectory(goal, seed)
                    except RuntimeError:
                        continue  # esta semilla no converge, probar la siguiente
                    if _first_trajectory_collision(
                        candidate, self._kinematics, self._link_radius_meters, self._structural_exclusions
                    ) is None:
                        # El origen real de la trayectoria sigue siendo
                        # current_configuration -- la semilla "nudged" fue
                        # solo un truco para que Newton-Raphson converja a
                        # otra rama, no forma parte del resultado.
                        return Trajectory.create(
                            [current_configuration, candidate.waypoints[-1]]
                        ).value
                nudge_degrees += self._nudge_step_degrees

        index, link_a, link_b, penetration = collision
        raise SelfCollisionError(
            f"Waypoint {index}/{len(trajectory.waypoints) - 1}: autocolisión "
            f"entre el eslabón {link_a} y el eslabón {link_b} "
            f"(penetración {penetration * 100:.1f}cm) -- ningún desplazamiento de "
            f"muñeca hasta ±{self._max_nudge_degrees:.0f}° encontró una rama libre; "
            "trayectoria rechazada, no se ha mandado nada al robot."
        )

    @staticmethod
    def _wrist_joint_pair(configuration: JointConfiguration) -> Optional[Tuple[str, str]]:
        """`(positions[-3].joint_name, positions[-1].joint_name)` -- los
        dos joints exteriores de una muñeca esférica de 3 ejes, saltando
        el del medio (la propia singularidad, `joint5` en el CR5) -- ver
        docstring del módulo. `None` si hay menos de 3 joints (no hay
        muñeca de 3 ejes que explotar)."""
        positions = configuration.positions
        if len(positions) < 3:
            return None
        return positions[-3].joint_name, positions[-1].joint_name


def _nudge_configuration(
    configuration: JointConfiguration,
    joint_a: str,
    joint_b: str,
    delta_degrees: float,
) -> JointConfiguration:
    """`configuration` con `joint_a` desplazado `+delta_degrees` y
    `joint_b` desplazado `-delta_degrees` -- el resto sin tocar. Usado como
    semilla de reintento de IK, no como resultado final (ver
    `SelfCollisionAvoidingPlanningAdapter.compute_trajectory`)."""
    delta_radians = math.radians(delta_degrees)
    positions = []
    for position in configuration.positions:
        if position.joint_name == joint_a:
            positions.append(JointPosition(joint_a, position.angle_radians + delta_radians))
        elif position.joint_name == joint_b:
            positions.append(JointPosition(joint_b, position.angle_radians - delta_radians))
        else:
            positions.append(position)
    return JointConfiguration.create(positions).value
