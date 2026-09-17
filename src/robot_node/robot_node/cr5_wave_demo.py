"""Hace que el CR5 FÍSICO REAL SALUDE con la mano: la punta (tip) recorre
un ARCO suave de lado a lado -- más alto en el centro que en los extremos
-- mientras la herramienta, inclinada HACIA ARRIBA, bascula en fase con el
desplazamiento. Misma geometría ya verificada en CoppeliaSim
(`commander/cr5_wave_sim_demo.py`), mandando cada waypoint a
`Cr5RealRobotAdapter.set_joints` por la MISMA conexión TCP.

Pensado para un CR5 con PINZA montada, que hace de "mano": de ahí la
inclinación hacia arriba y el recorrido curvo (14/09, a petición del
usuario: "quedaría más natural").

Resumen de la geometría -- el razonamiento completo, con los números y las
alternativas descartadas, está en el docstring del demo de simulación:

  - NO se saluda desde la home: ahí el tip está a 0.933m del hombro y el
    alcance de catálogo del CR5 es 0.9m -- la home está en el borde del
    espacio alcanzable, y un saludo necesita margen LATERAL. Es el mismo
    hallazgo que ya obligó a rediseñar el semicírculo ("la home no puede
    subir, solo bajar"), ahora con un número que lo explica.
  - `--start-degrees` (por defecto 0 -45 90 45 25 -90) es la postura de
    saludo: codo doblado 90° (tip a 0.647m del hombro, 25cm de margen) y
    muñeca puesta de forma que **el QUINTO valor, `joint5`, ES el ángulo
    de inclinación hacia arriba de la herramienta**. Eso funciona porque
    `joint4`=+45/`joint6`=-90 conjugan la rotación de `joint5` (eje Z del
    mundo) y la convierten en una rotación sobre el eje X del mundo, que
    es el cabeceo: Ry(-90)·Rz(β)·Ry(90) = Rx(-β). Ningún joint suelto da
    cabeceo en esta postura; esa conjugación sí, exactamente y sin IK.
  - El saludo es traslación de ±`--amplitude-meters` en X (la
    izquierda-derecha de quien mire al robot de frente, desde -Y), ARCO de
    `--arc-sag-meters` de flecha (z = z0 - sag·s², centro más alto que los
    extremos: la curva que traza una mano pivotando sobre un punto por
    debajo) y basculación de ±`--tilt-degrees` sobre el eje Y del mundo --
    la línea de visión del observador, así que la mano se mece en SU plano
    de imagen. Todo en fase.
  - Con los valores por defecto, `joint1` se queda a 0° y `joint5`/`joint6`
    CLAVADOS en la inclinación durante todo el saludo: el gesto entero lo
    hacen `joint2`/`joint3`/`joint4`. El robot no gira el "cuerpo" ni
    retuerce la muñeca, solo mece el brazo con la mano fija apuntando
    hacia arriba.

A diferencia del círculo/semicírculo, este demo NO construye la
trayectoria alrededor de la pose ACTUAL del robot: la postura de saludo es
parte del gesto (es la que da el margen de alcance Y la inclinación de la
mano), así que primero se va a ella con un único `MovJ` -- mismo patrón, y
mismos avisos, que `cr5_go_home_demo.py` -- y solo después se saluda. El
recorrido de CADA joint hasta esa postura se muestra antes de pedir
confirmación, y se avisa si alguno supera `--warn-threshold-degrees`.

Toda la IK (la del saludo entero) se resuelve ANTES de conectar siquiera
con el robot: la postura de saludo se conoce analíticamente, no hay que
leerla del robot, así que se puede abortar sin haber tocado nada si algún
punto no converge o no tiene rama libre de autocolisión. El
`KinematicsPort` que resuelve cada punto es un
`SelfCollisionAvoidingPlanningAdapter` desde el principio.

Autocolisión: medido en el recorrido completo, la distancia mínima entre
eslabones no adyacentes es 0.116m -- la MISMA que en la home, y muy lejos
de los 0.077m a los que saltó la alarma real [76] del fabricante durante
`cr5_semicircle_demo.py`. Es, de los recorridos cartesianos de este repo,
el más holgado.

Aviso de seguridad explícito (Bloque 0 #114, todavía sin resolver):
`Cr5RealRobotAdapter` valida el límite mecánico ABSOLUTO de cada joint,
pero NO valida la velocidad/salto entre un waypoint y el siguiente -- se
muestra el máximo real por joint antes de pedir confirmación (con los
valores por defecto: ~7°, más pequeño que en el círculo).

Espera explícitamente a que `RobotMode()` vuelva a 5
(`_wait_until_robot_idle`) tras el acercamiento y tras el saludo, antes de
leer la posición o des-energizar -- mismo hallazgo y misma corrección del
08/09 que `cr5_semicircle_demo.py`/`cr5_circle_demo.py`: con `cp=50` el
controlador sigue procesando la cola de `MovJ` a su propio ritmo, y sin
esta espera el robot se queda parado a mitad del gesto, sin ningún error.

Uso:
    ros2 run robot_node cr5_wave_demo --host <IP_DEL_ROBOT>

    Opcional: --joint-names (si difieren de los de robot_node.yaml),
    --start-degrees (6 valores; el QUINTO es la inclinación hacia arriba),
    --amplitude-meters (por defecto 0.15), --arc-sag-meters (por defecto
    0.05), --tilt-degrees (por defecto 25), --cycles (por defecto 3),
    --points-per-cycle (por defecto 16), --waypoint-pause-seconds (por
    defecto 0.2), --warn-threshold-degrees (por defecto 30).
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
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene

from robot_node.adapters._cr5_protocol import CONTROLLABLE_ROBOT_MODES, Cr5ProtocolError
from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

_DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Ver docstring: joint4=+45/joint6=-90 conjugan la rotación de joint5 para
# convertirla en cabeceo, así que el QUINTO valor es directamente el ángulo
# de inclinación hacia arriba de la herramienta.
_DEFAULT_START_DEGREES = [0.0, -45.0, 90.0, 45.0, 25.0, -90.0]
_DEFAULT_AMPLITUDE_METERS = 0.15
_DEFAULT_ARC_SAG_METERS = 0.05
_DEFAULT_TILT_DEGREES = 25.0
_DEFAULT_CYCLES = 3
_DEFAULT_POINTS_PER_CYCLE = 16
_DEFAULT_WAYPOINT_PAUSE_SECONDS = 0.2
_DEFAULT_WARN_THRESHOLD_DEGREES = 30.0
_SHOULDER_HEIGHT_METERS = 0.147


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--joint-names", nargs=6, default=_DEFAULT_JOINT_NAMES)
    parser.add_argument(
        "--start-degrees",
        nargs=6,
        type=float,
        default=_DEFAULT_START_DEGREES,
        help="Postura de saludo. El QUINTO valor (joint5) es el ángulo de "
        "inclinación hacia arriba de la herramienta -- ver docstring.",
    )
    parser.add_argument("--amplitude-meters", type=float, default=_DEFAULT_AMPLITUDE_METERS)
    parser.add_argument("--arc-sag-meters", type=float, default=_DEFAULT_ARC_SAG_METERS)
    parser.add_argument("--tilt-degrees", type=float, default=_DEFAULT_TILT_DEGREES)
    parser.add_argument("--cycles", type=int, default=_DEFAULT_CYCLES)
    parser.add_argument("--points-per-cycle", type=int, default=_DEFAULT_POINTS_PER_CYCLE)
    parser.add_argument(
        "--waypoint-pause-seconds", type=float, default=_DEFAULT_WAYPOINT_PAUSE_SECONDS
    )
    parser.add_argument(
        "--warn-threshold-degrees", type=float, default=_DEFAULT_WARN_THRESHOLD_DEGREES
    )
    return parser.parse_args()


def _format_degrees(joint_names: List[str], configuration: JointConfiguration) -> str:
    return ", ".join(
        f"{name}={math.degrees(configuration.angle_of(name)):.2f}°"
        for name in joint_names
    )


def _rotated_about_world_y(pose: Pose, angle_radians: float) -> Pose:
    """`pose` con su orientación rotada `angle_radians` alrededor del eje Y
    del MUNDO (la posición no cambia), como producto de cuaterniones
    q_y ⊗ q_pose -- premultiplicar es justo lo que rota en el marco fijo
    del mundo, no en el marco local de la herramienta.

    Idéntico a `commander/cr5_wave_sim_demo.py::_rotated_about_world_y`.
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
    """Idéntico a `commander/cr5_wave_sim_demo.py::_wave_points`, pero
    parametrizado sobre la `start_pose` de la postura pedida por
    `--start-degrees`. Puntos del saludo alrededor de `start_pose`, con fase
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
    `cr5_circle_demo.py::_snap_to_exact_start_if_needed`: la
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



def _confirm(prompt: str) -> bool:
    answer = input(f"{prompt} Escribe SI (en mayúsculas) para continuar: ")
    return answer.strip() == "SI"


def _wait_until_robot_idle(adapter: Cr5RealRobotAdapter, timeout_seconds: float = 20.0) -> None:
    """Espera a que `RobotMode()` vuelva a 5 (habilitado e inactivo) antes
    de leer la posición o des-energizar -- ver el hallazgo del 08/09 en
    `cr5_semicircle_demo.py`: con `cp=50` el controlador sigue procesando
    la cola de `MovJ` a su propio ritmo real, y sin esto tanto la lectura
    como `DisableRobot()` pueden llegar con el robot a mitad de camino."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        mode, _ = adapter.get_robot_mode()
        if mode == 5:
            return
        time.sleep(0.1)
    print(f"Aviso: RobotMode() no volvió a 5 en {timeout_seconds}s -- leyendo/cerrando igualmente.")


def main() -> None:
    args = _parse_args()
    joint_names = list(args.joint_names)

    kinematics = PoeKinematicsAdapter(steps=1)
    planner = SelfCollisionAvoidingPlanningAdapter(kinematics)

    wave_start = JointConfiguration.create(
        [
            JointPosition(name, math.radians(degrees))
            for name, degrees in zip(joint_names, args.start_degrees)
        ]
    ).value
    start_pose = kinematics.forward_kinematics(wave_start)
    shoulder_distance = math.hypot(
        math.hypot(start_pose.x, start_pose.y), start_pose.z - _SHOULDER_HEIGHT_METERS
    )

    print("=" * 70)
    print("CR5 FÍSICO -- SALUDO: mano inclinada hacia arriba, arco de lado a lado.")
    print(f"Host: {args.host}")
    print(
        f"Postura de saludo: {_format_degrees(joint_names, wave_start)}\n"
        f"  tip=(x={start_pose.x:.4f}, y={start_pose.y:.4f}, z={start_pose.z:.4f}), "
        f"a {shoulder_distance:.3f}m del hombro (alcance del CR5 ~0.9m).\n"
        f"  Inclinación de la herramienta hacia arriba: {args.start_degrees[4]:.0f}° (joint5)."
    )
    print(
        f"Saludo: ±{args.amplitude_meters:.2f}m de lado a lado en arco de "
        f"{args.arc_sag_meters:.2f}m de flecha, con ±{args.tilt_degrees:.0f}° de "
        f"basculación, {args.cycles} ciclos de {args.points_per_cycle} puntos."
    )
    print("Ten una mano cerca del botón físico de parada de emergencia.")
    print("=" * 70)

    print("\nResolviendo IK de TODO el saludo (sin conectar todavía al robot)...")
    points = _wave_points(
        start_pose,
        args.amplitude_meters,
        args.arc_sag_meters,
        args.tilt_degrees,
        args.cycles,
        args.points_per_cycle,
    )
    configurations = [wave_start]
    current = wave_start
    for index, point in enumerate(points[1:], start=1):
        try:
            segment = planner.compute_trajectory(point, current, Scene.empty())
        except SelfCollisionError as error:
            print(
                f"\nAutocolisión sin rama libre en el punto {index}/{len(points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Cancelado -- ni siquiera se ha conectado al robot.")
            return
        except RuntimeError as error:
            print(
                f"\nIK no convergió en el punto {index}/{len(points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Cancelado -- ni siquiera se ha conectado al robot.")
            return
        current = segment.waypoints[-1]
        configurations.append(current)

    _snap_to_exact_start_if_needed(configurations, joint_names)

    max_step_degrees = {name: 0.0 for name in joint_names}
    for previous, following in zip(configurations, configurations[1:]):
        for name in joint_names:
            step = abs(math.degrees(following.angle_of(name) - previous.angle_of(name)))
            max_step_degrees[name] = max(max_step_degrees[name], step)
    print(
        f"{len(configurations)} configuraciones resueltas "
        f"({len(configurations) - 1} MovJ para el saludo)."
    )
    print("Paso máximo entre dos waypoints consecutivos, por joint:")
    for name in joint_names:
        print(f"  {name}: {max_step_degrees[name]:.2f}°")

    adapter = Cr5RealRobotAdapter(args.host, joint_names=joint_names)
    try:
        print("\nPaso 1/5 -- consultando el estado del robot (RobotMode)...")
        mode, description = adapter.get_robot_mode()
        print(f"Estado actual: {mode} -- {description}")
        if mode not in CONTROLLABLE_ROBOT_MODES:
            print(
                "\nSegún el manual, el robot NO está en un estado desde el que "
                "se garantice poder pedir el control por TCP."
            )
            if not _confirm("¿Quieres intentar continuar de todas formas?"):
                print("Cancelado por el usuario. No se ha mandado ningún comando de control.")
                return

        print("\nPaso 2/5 -- leyendo la posición actual (no mueve nada)...")
        current_configuration = adapter.get_current_configuration()
        print(f"Posición actual: {_format_degrees(joint_names, current_configuration)}")

        deltas_degrees = [
            math.degrees(wave_start.angle_of(name) - current_configuration.angle_of(name))
            for name in joint_names
        ]
        print("\nRecorrido por joint hasta la postura de saludo:")
        for name, delta in zip(joint_names, deltas_degrees):
            marker = (
                " <- MÁS DE LO HABITUAL"
                if abs(delta) > args.warn_threshold_degrees
                else ""
            )
            print(f"  {name}: {delta:+.2f}°{marker}")
        if any(abs(delta) > args.warn_threshold_degrees for delta in deltas_degrees):
            print(
                f"\nAl menos un joint recorre más de {args.warn_threshold_degrees:.0f}° -- "
                "MovJ usa la velocidad/aceleración por defecto del propio robot, "
                "así que ese recorrido puede notarse rápido. Confirma solo si la "
                "zona alrededor del robot está despejada."
            )

        print("\nPaso 3/5 -- acercamiento a la postura de saludo (un único MovJ).")
        if not _confirm("¿Confirmas ESTE movimiento a la postura de saludo, ahora mismo?"):
            print("Cancelado por el usuario. No se ha mandado ningún movimiento.")
            return
        adapter.set_joints(wave_start)
        print("MovJ enviado. Esperando a que el robot llegue...")
        _wait_until_robot_idle(adapter)
        reached = adapter.get_current_configuration()
        print(f"Postura alcanzada: {_format_degrees(joint_names, reached)}")

        print(
            "\nRecuerda: Cr5RealRobotAdapter valida el límite mecánico absoluto "
            "de cada joint, pero NO valida la velocidad/salto entre waypoints "
            "consecutivos (Bloque 0 #114, sin resolver) -- los pasos de arriba "
            "son pequeños, pero la comprobación de que tiene sentido la haces tú."
        )
        if not _confirm(
            f"\nPaso 4/5 -- ¿confirmas SALUDAR ahora ({len(configurations) - 1} "
            "MovJ seguidos por la misma conexión)?"
        ):
            print("Cancelado por el usuario. No se ha mandado el saludo.")
            return

        print("\nSaludando...")
        for index, configuration in enumerate(configurations[1:], start=1):
            adapter.set_joints(configuration)
            print(f"  MovJ {index}/{len(configurations) - 1} enviado.")
            time.sleep(args.waypoint_pause_seconds)

        print("\nSecuencia enviada. Esperando a que el robot termine de moverse...")
        _wait_until_robot_idle(adapter)

        print("\nPaso 5/5 -- releyendo la posición final...")
        after = adapter.get_current_configuration()
        print(f"Posición final real: {_format_degrees(joint_names, after)}")
        final_pose = kinematics.forward_kinematics(after)
        print(
            f"Pose cartesiana final real: (x={final_pose.x:.4f}, "
            f"y={final_pose.y:.4f}, z={final_pose.z:.4f}) -- postura de saludo: "
            f"(x={start_pose.x:.4f}, y={start_pose.y:.4f}, z={start_pose.z:.4f})"
        )
    except Cr5ProtocolError as error:
        print(f"\nFallo de protocolo/conexión: {error}")
        print("Revisa red/puertos antes de reintentar.")
    finally:
        if adapter.is_enabled:
            print("\nDes-energizando el robot (DisableRobot) antes de salir...")
            try:
                adapter.disable()
                print("Robot des-energizado correctamente.")
            except Cr5ProtocolError as disable_error:
                print(f"No se pudo des-energizar automáticamente: {disable_error}")
                print(
                    "Des-energízalo a mano desde el panel/teach pendant antes de "
                    "dejar el robot sin vigilancia."
                )
        adapter.close()


if __name__ == "__main__":
    main()
