"""Hace que el CR5 SALUDE con la mano en CoppeliaSim: la punta (tip)
recorre un ARCO suave de lado a lado -- más alto en el centro que en los
extremos, como una mano que pivota sobre la muñeca -- mientras la
herramienta, inclinada HACIA ARRIBA, bascula en fase con el
desplazamiento.

Pensado para un CR5 con PINZA montada, que hace de "mano": por eso la
herramienta no apunta horizontal sino inclinada hacia arriba, y por eso el
recorrido es curvo y no una línea recta (14/09, a petición del usuario:
"quedaría más natural").

## Por qué NO se saluda desde la home

En la home (los 6 joints a 0°) el brazo está COMPLETAMENTE EXTENDIDO hacia
arriba y el tip queda a 0.933m del hombro -- y el alcance de catálogo del
CR5 es 0.9m. La home está literalmente en el borde del espacio alcanzable.
Eso explica de una vez el hallazgo del 08/09 ("la home no puede subir,
solo bajar", ver [[PoeKinematicsAdapter]]): tampoco puede moverse de lado
manteniendo la altura, porque cualquier punto con el mismo z y distinto x
queda TODAVÍA más lejos del hombro. Un saludo necesita margen lateral, así
que hay que partir de una postura retraída.

## La postura de saludo, y por qué joint5 es el ángulo de inclinación

`_DEFAULT_START_DEGREES` = (0, -45, 90, 45, 25, -90):

  - `joint2`/`joint3` = -45°/90° -- codo doblado 90°, que es lo que da el
    margen: el tip queda a 0.647m del hombro, 25cm por debajo del alcance
    máximo.
  - `joint4` = +45° y `joint6` = -90° NO son arbitrarios: en esta postura
    el eje de `joint4`/`joint6` es el eje Y del mundo y el de `joint5` es
    el eje Z, y **conjugar una rotación sobre Z por ±90° sobre Y la
    convierte en una rotación sobre X**: Ry(-90)·Rz(β)·Ry(90) = Rx(-β).
    El cabeceo ("mirar hacia arriba") ES una rotación sobre el eje X del
    mundo, y ningún joint suelto del CR5 lo da en esta postura -- pero esa
    conjugación sí, exactamente y sin IK de por medio (verificado
    numéricamente: error de orientación ~1e-6 frente al objetivo).
  - Consecuencia práctica y muy cómoda: **`joint5` ES el ángulo de
    inclinación hacia arriba**. `--start-degrees 0 -45 90 45 35 -90` da 35°
    de cabeceo, y nada más cambia.

Alternativa descartada (14/09): pedir la pose inclinada por IK (rotar la
pose "plana" sobre X y dejar que Newton-Raphson encuentre la postura). Sí
converge, pero a una rama contorsionada (`joint1`=-9°, `joint6`=-69°)
desde la que el saludo daba saltos de **hasta 51° entre waypoints
consecutivos** -- la IK iba saltando de rama a lo largo del barrido. Fijar
la postura analíticamente y dejar que la IK solo siga deltas pequeños baja
ese máximo a ~7°.

## La geometría del gesto

  - Quien mire al robot de frente está en -Y (es hacia donde apunta la
    herramienta). Para esa persona, su izquierda-derecha es el eje X del
    mundo -- y el eje X está justo en el plano en el que bascula el brazo
    (`joint2`/`joint3`/`joint4`). De ahí que "de lado a lado" sea oscilar
    en X, sin necesidad de mover `joint1`.
  - **Arco, no línea recta**: `z = z0 - sag·sin²(...)`, o sea el centro
    del barrido más alto que los extremos por `--arc-sag-meters`. Es la
    forma que traza una mano que pivota sobre un punto por DEBAJO (la
    muñeca/el codo), que es lo que hace que se lea como un saludo y no
    como limpiar un cristal.
  - **Basculación** rotando sobre el eje Y del mundo -- que es la línea de
    visión del observador, así que la rotación ocurre en SU plano de
    imagen: la mano se mece a izquierda y derecha desde su punto de vista,
    sea cual sea la inclinación hacia arriba que lleve.
  - Traslación, arco y basculación van EN FASE: cuando la mano va hacia la
    derecha del observador, también baja un poco y se inclina hacia la
    derecha. En contrafase el gesto se lee como "limpiar un cristal".

Resultado (verificado, con los valores por defecto): `joint1` se queda a
0°, y `joint5`/`joint6` se quedan CLAVADOS en la inclinación (25°/-90°)
durante todo el saludo -- el gesto entero lo hacen `joint2`/`joint3`/
`joint4`, el plano del brazo. El robot no gira el "cuerpo" ni retuerce la
muñeca: solo mece el brazo, con la mano fija apuntando hacia arriba.

Igual que el círculo (`cr5_circle_sim_demo.py`), el `KinematicsPort` que
resuelve cada punto es un `SelfCollisionAvoidingPlanningAdapter` desde el
principio. El saludo resulta MUY holgado de autocolisión: la distancia
mínima entre eslabones no adyacentes es 0.116m en todo el recorrido --
la misma que en la home, y muy lejos de los 0.077m a los que saltó la
alarma real [76] del fabricante durante el semicírculo (ver
[[SelfCollisionAwarePlanningAdapter]]).

Por construcción el saludo CIERRA sobre la postura de partida
(sin(2π·ciclos) = 0), con un residuo medido de 0.01° -- muy por debajo del
`_SNAP_TOLERANCE_DEGREES` de 0.5° que obligó a
`_snap_to_exact_start_if_needed` en el círculo, así que ahí ese waypoint
extra normalmente ni se dispara; se conserva por si se cambian los
parámetros.

Uso: `ros2 run commander cr5_wave_sim_demo` (lanza CoppeliaSim solo si
hace falta). En CoppeliaSim: el brazo va a la postura de saludo (mano
inclinada hacia arriba) y mece la mano en arco, de lado a lado, tres
veces.

    Opcional: --amplitude-meters (por defecto 0.15), --arc-sag-meters (por
    defecto 0.05 -- cuánto más bajos los extremos que el centro),
    --tilt-degrees (por defecto 25), --cycles (por defecto 3),
    --points-per-cycle (por defecto 16), --start-degrees (6 valores; el
    QUINTO es la inclinación hacia arriba, ver arriba),
    --step-pause-seconds (por defecto 0.06), --port (por defecto 23000).
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List

from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from controller_node.adapters.self_collision_planning_adapter import (
    SelfCollisionAvoidingPlanningAdapter,
    SelfCollisionError,
)
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene, Trajectory

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_HOME = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value
# Ver el docstring del módulo: joint4=+45/joint6=-90 conjugan la rotación
# de joint5 para convertirla en cabeceo, así que el QUINTO valor es
# directamente el ángulo de inclinación hacia arriba de la herramienta.
_DEFAULT_START_DEGREES = (0.0, -45.0, 90.0, 45.0, 25.0, -90.0)
_DEFAULT_AMPLITUDE_METERS = 0.15
_DEFAULT_ARC_SAG_METERS = 0.05
_DEFAULT_TILT_DEGREES = 25.0
_DEFAULT_CYCLES = 3
_DEFAULT_POINTS_PER_CYCLE = 16
_DEFAULT_STEP_PAUSE_SECONDS = 0.06
_SHOULDER_HEIGHT_METERS = 0.147


def _rotated_about_world_y(pose: Pose, angle_radians: float) -> Pose:
    """`pose` con su orientación rotada `angle_radians` alrededor del eje Y
    del MUNDO (la posición no cambia), como producto de cuaterniones
    q_y ⊗ q_pose -- premultiplicar es justo lo que rota en el marco fijo
    del mundo, no en el marco local de la herramienta.

    El eje Y del mundo es la línea de visión de quien mira al robot de
    frente, así que esta rotación ocurre en SU plano de imagen: la mano se
    mece a su izquierda y a su derecha, lleve la inclinación hacia arriba
    que lleve. Ver docstring del módulo.

    Se hace con cuaterniones a mano y no reusando los helpers de
    `poe_adapter` porque esos son privados de ese adaptador
    (`_pose_to_matrix`/`_matrix_to_pose`): `Pose` ya lleva el cuaternión, y
    esto es geometría de la trayectoria, no cinemática del robot."""
    half = angle_radians / 2.0
    ax, ay, az, aw = 0.0, math.sin(half), 0.0, math.cos(half)
    bx, by, bz, bw = pose.qx, pose.qy, pose.qz, pose.qw
    return Pose(
        x=pose.x,
        y=pose.y,
        z=pose.z,
        qx=aw * bx + ax * bw + ay * bz - az * by,
        qy=aw * by - ax * bz + ay * bw + az * bx,
        qz=aw * bz + ax * by - ay * bx + az * bw,
        qw=aw * bw - ax * bx - ay * by - az * bz,
    )


def _wave_points(
    start_pose: Pose,
    amplitude: float,
    arc_sag: float,
    tilt_degrees: float,
    cycles: int,
    points_per_cycle: int,
) -> List[Pose]:
    """Puntos del saludo alrededor de `start_pose`, con fase
    `s = sin(2πt)`:

      - x = x0 + amplitude·s        (de lado a lado)
      - z = z0 - arc_sag·s²          (ARCO: centro más alto que extremos)
      - y = y0                       (constante)
      - orientación = la de `start_pose` rotada tilt_degrees·s sobre el
        eje Y del mundo (basculación, en fase con lo anterior)

    `s²` en la altura y no `s` es lo que hace que sea un arco simétrico y
    no una rampa: el tip está arriba del todo en el centro del barrido
    (s=0) y `arc_sag` más abajo en AMBOS extremos (s=±1), que es la curva
    que traza una mano pivotando sobre un punto por debajo.

    El primer punto (t=0) es `start_pose` exacta y el último (t=cycles)
    también, porque sin(2π·cycles) = 0 -- el saludo cierra sobre su propia
    postura de partida por construcción."""
    points = []
    for i in range(cycles * points_per_cycle + 1):
        phase = math.sin(2 * math.pi * i / points_per_cycle)
        displaced = Pose(
            x=start_pose.x + amplitude * phase,
            y=start_pose.y,
            z=start_pose.z - arc_sag * phase * phase,
            qx=start_pose.qx,
            qy=start_pose.qy,
            qz=start_pose.qz,
            qw=start_pose.qw,
        )
        points.append(
            _rotated_about_world_y(displaced, math.radians(tilt_degrees) * phase)
        )
    return points


_SNAP_TOLERANCE_DEGREES = 0.5


def _snap_to_exact_start_if_needed(
    configurations: List[JointConfiguration], joint_names: List[str]
) -> None:
    """Mismo mecanismo (y mismo motivo) que
    `cr5_circle_sim_demo.py::_snap_to_exact_start_if_needed`: la
    redundancia joint4/joint6 que explota
    `SelfCollisionAvoidingPlanningAdapter` puede dejar el cierre a 1-2° de
    la postura de partida aunque la punta sí vuelva a su sitio.

    En el saludo, medido, el residuo es de 0.01° -- muy por debajo de la
    tolerancia, así que normalmente NO se dispara; se conserva por si se
    cambian amplitud/arco/basculación/ciclos y el recorrido deja de cerrar
    tan limpio."""
    if not configurations:
        return
    start, last = configurations[0], configurations[-1]
    max_delta = max(
        abs(math.degrees(last.angle_of(name) - start.angle_of(name)))
        for name in joint_names
    )
    if max_delta <= _SNAP_TOLERANCE_DEGREES:
        return
    print(
        f"Cierre del saludo a {max_delta:.2f}° de la postura de partida "
        "(redundancia de muñeca) -- añadiendo un waypoint final a la "
        "postura de partida exacta."
    )
    configurations.append(start)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--amplitude-meters", type=float, default=_DEFAULT_AMPLITUDE_METERS)
    parser.add_argument("--arc-sag-meters", type=float, default=_DEFAULT_ARC_SAG_METERS)
    parser.add_argument("--tilt-degrees", type=float, default=_DEFAULT_TILT_DEGREES)
    parser.add_argument("--cycles", type=int, default=_DEFAULT_CYCLES)
    parser.add_argument("--points-per-cycle", type=int, default=_DEFAULT_POINTS_PER_CYCLE)
    parser.add_argument(
        "--start-degrees",
        nargs=6,
        type=float,
        default=list(_DEFAULT_START_DEGREES),
        help="Postura de saludo. El QUINTO valor (joint5) es el ángulo de "
        "inclinación hacia arriba de la herramienta -- ver docstring.",
    )
    parser.add_argument("--step-pause-seconds", type=float, default=_DEFAULT_STEP_PAUSE_SECONDS)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    return parser.parse_args()


def run(
    amplitude: float = _DEFAULT_AMPLITUDE_METERS,
    arc_sag: float = _DEFAULT_ARC_SAG_METERS,
    tilt_degrees: float = _DEFAULT_TILT_DEGREES,
    cycles: int = _DEFAULT_CYCLES,
    points_per_cycle: int = _DEFAULT_POINTS_PER_CYCLE,
    start_degrees: List[float] = list(_DEFAULT_START_DEGREES),
    step_pause_seconds: float = _DEFAULT_STEP_PAUSE_SECONDS,
    port: int = _ZMQ_PORT,
) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_wave_sim_demo_{port}")
    robot = build_cr5_scene(port=port, initial_configuration=_HOME, scene=Scene.empty())

    kinematics = PoeKinematicsAdapter(steps=1)
    planner = SelfCollisionAvoidingPlanningAdapter(kinematics)

    start_configuration = JointConfiguration.create(
        [
            JointPosition(name, math.radians(degrees))
            for name, degrees in zip(_JOINT_NAMES, start_degrees)
        ]
    ).value
    start_pose = kinematics.forward_kinematics(start_configuration)
    shoulder_distance = math.hypot(
        math.hypot(start_pose.x, start_pose.y),
        start_pose.z - _SHOULDER_HEIGHT_METERS,
    )
    print(
        f"Postura de saludo: tip=(x={start_pose.x:.4f}, y={start_pose.y:.4f}, "
        f"z={start_pose.z:.4f}), a {shoulder_distance:.3f}m del hombro "
        f"(alcance del CR5 ~0.9m). Inclinación de la herramienta hacia "
        f"arriba: {start_degrees[4]:.0f}° (joint5)."
    )
    print(
        f"Saludo: ±{amplitude:.2f}m de lado a lado en arco de {arc_sag:.2f}m "
        f"de flecha, con ±{tilt_degrees:.0f}° de basculación, "
        f"{cycles} ciclos de {points_per_cycle} puntos."
    )

    print("\nPaso 1/3 -- yendo a la postura de saludo desde la home...")
    # Interpolación directa en espacio de articulaciones, sin IK: la postura
    # de saludo ya se conoce como `JointConfiguration` (viene de
    # `--start-degrees`), así que resolver la IK de su pose sería dar un
    # rodeo -- y podría además devolver OTRA rama de muñeca que llegue a la
    # misma pose, no la postura concreta que se validó (justo el problema
    # que documenta el docstring del módulo). Es lo mismo que hace el demo
    # real con un único MovJ; aquí se trocea solo para verlo moverse.
    for configuration in Trajectory.straight_line(_HOME, start_configuration, 25).waypoints:
        robot.set_joints(configuration)
        time.sleep(step_pause_seconds)

    print("\nPaso 2/3 -- resolviendo IK de todos los puntos del saludo...")
    points = _wave_points(
        start_pose, amplitude, arc_sag, tilt_degrees, cycles, points_per_cycle
    )
    robot.mark_goal(points[points_per_cycle // 4])

    configurations = [start_configuration]
    current = start_configuration
    for index, point in enumerate(points[1:], start=1):
        try:
            segment = planner.compute_trajectory(point, current, Scene.empty())
        except SelfCollisionError as error:
            print(
                f"Autocolisión sin rama libre en el punto {index}/{len(points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el saludo aquí -- se anima lo ya resuelto.")
            break
        except RuntimeError as error:
            print(
                f"IK no convergió en el punto {index}/{len(points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el saludo aquí -- se anima lo ya resuelto.")
            break
        current = segment.waypoints[-1]
        configurations.append(current)

    _snap_to_exact_start_if_needed(configurations, _JOINT_NAMES)

    print(f"\nPaso 3/3 -- {len(configurations)} configuraciones. Saludando...")
    for configuration in configurations:
        robot.set_joints(configuration)
        time.sleep(step_pause_seconds)

    final_pose = kinematics.forward_kinematics(configurations[-1])
    print(
        f"Listo. Punto final: (x={final_pose.x:.4f}, y={final_pose.y:.4f}, "
        f"z={final_pose.z:.4f}) -- partida del saludo: (x={start_pose.x:.4f}, "
        f"y={start_pose.y:.4f}, z={start_pose.z:.4f})."
    )


def main() -> None:
    args = _parse_args()
    run(
        amplitude=args.amplitude_meters,
        arc_sag=args.arc_sag_meters,
        tilt_degrees=args.tilt_degrees,
        cycles=args.cycles,
        points_per_cycle=args.points_per_cycle,
        start_degrees=list(args.start_degrees),
        step_pause_seconds=args.step_pause_seconds,
        port=args.port,
    )


if __name__ == "__main__":
    main()
