"""Traza con la punta (tip) del CR5 FÍSICO REAL una CIRCUNFERENCIA
COMPLETA (360°, no solo el semicírculo de `cr5_semicircle_demo.py`)
antihoraria, en el plano Y=constante, ya verificada en CoppeliaSim
(`commander/cr5_circle_sim_demo.py`) -- geometría idéntica, mandando cada
waypoint a `Cr5RealRobotAdapter.set_joints` por la MISMA conexión TCP.

Misma base que `cr5_semicircle_demo.py`: centro de la circunferencia
DIRECTAMENTE DEBAJO de la pose de partida real, home en theta=90°, pero
aquí se barre 90°→450° (=90°+360°), la vuelta entera -- cierra de nuevo
sobre el punto de partida. Por construcción, z(theta) nunca supera la
altura de partida (ver docstring de `cr5_circle_sim_demo.py`), así que el
círculo completo se queda dentro de la misma región alcanzable ya
validada para el semicírculo.

Toda la IK se resuelve ANTES de mandar nada al robot: se lee la posición
ACTUAL real del robot (lo normal es correr esto justo después de
`cr5_go_home_demo.py`) y se calcula el círculo desde esa pose real. El
`KinematicsPort` que resuelve cada punto es un
`SelfCollisionAvoidingPlanningAdapter` desde el principio (no
`PoeKinematicsAdapter` a secas) -- si algún punto no converge o no tiene
rama libre de autocolisión, se aborta ANTES de tocar el robot.

Es una prueba MÁS EXIGENTE que el semicírculo de "varios objetivos
consecutivos sin cortar la conexión" (Bloque 0): el doble de MovJ por la
misma conexión, cerrando en el mismo punto de partida.

Aviso de seguridad explícito (Bloque 0 #114, todavía sin resolver):
`Cr5RealRobotAdapter` valida el límite mecánico ABSOLUTO de cada joint,
pero NO valida la velocidad/salto entre un waypoint y el siguiente -- se
muestra el máximo real por joint antes de pedir confirmación.

**Cierre prematuro (08/09), corregido:** espera explícitamente a que
`RobotMode()` vuelva a 5 (`_wait_until_robot_idle`) antes de leer la
posición final y des-energizar -- mismo hallazgo y misma corrección que
`cr5_semicircle_demo.py` (ver ahí el detalle): sin esto, con `cp=50` el
robot puede seguir procesando la cola de `MovJ` cuando ya hemos leído/
desenergizado, dejándolo detenido a mitad del círculo sin ningún error.

Uso:
    ros2 run robot_node cr5_circle_demo --host <IP_DEL_ROBOT>

    Opcional: --joint-names (si difieren de los de robot_node.yaml),
    --radius-meters (por defecto 0.15, igual que en CoppeliaSim),
    --num-points (por defecto 19 -- 18 pasos de 20° cada uno),
    --waypoint-pause-seconds (por defecto 0.3).
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
from shared_kernel import JointConfiguration, Pose, Scene

from robot_node.adapters._cr5_protocol import CONTROLLABLE_ROBOT_MODES, Cr5ProtocolError
from robot_node.adapters.cr5_real_adapter import Cr5RealRobotAdapter

_DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_DEFAULT_RADIUS_METERS = 0.15
_DEFAULT_NUM_POINTS = 19
_DEFAULT_WAYPOINT_PAUSE_SECONDS = 0.3


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="IP del controlador del CR5.")
    parser.add_argument("--joint-names", nargs=6, default=_DEFAULT_JOINT_NAMES)
    parser.add_argument("--radius-meters", type=float, default=_DEFAULT_RADIUS_METERS)
    parser.add_argument("--num-points", type=int, default=_DEFAULT_NUM_POINTS)
    parser.add_argument(
        "--waypoint-pause-seconds", type=float, default=_DEFAULT_WAYPOINT_PAUSE_SECONDS
    )
    return parser.parse_args()


def _format_degrees(joint_names: List[str], configuration: JointConfiguration) -> str:
    return ", ".join(
        f"{name}={math.degrees(configuration.angle_of(name)):.2f}°"
        for name in joint_names
    )


def _circle_points(start_pose: Pose, radius: float, num_points: int) -> List[Pose]:
    """Idéntico a `commander/cr5_circle_sim_demo.py::_circle_points`, pero
    parametrizado sobre `start_pose` real -- de theta=90° a 450°
    (vuelta completa), antihorario, centro de la circunferencia justo
    debajo de `start_pose`."""
    center_x = start_pose.x
    center_z = start_pose.z - radius
    points = []
    for i in range(num_points):
        theta = math.pi / 2 + 2 * math.pi * i / (num_points - 1)
        points.append(
            Pose(
                x=center_x + radius * math.cos(theta),
                y=start_pose.y,
                z=center_z + radius * math.sin(theta),
                qx=start_pose.qx,
                qy=start_pose.qy,
                qz=start_pose.qz,
                qw=start_pose.qw,
            )
        )
    return points


def _confirm(prompt: str) -> bool:
    answer = input(f"{prompt} Escribe SI (en mayúsculas) para continuar: ")
    return answer.strip() == "SI"


def _wait_until_robot_idle(adapter: Cr5RealRobotAdapter, timeout_seconds: float = 15.0) -> None:
    """Espera a que `RobotMode()` vuelva a 5 (habilitado e inactivo) antes
    de leer la posición final o des-energizar -- ver el hallazgo del
    08/09 en `cr5_semicircle_demo.py` (mismo fallo, misma corrección):
    con `cp=50` el controlador sigue procesando la cola de `MovJ` a su
    propio ritmo real, y sin esto tanto la lectura final como
    `DisableRobot()` pueden llegar mientras el robot sigue a mitad de
    camino. Mismo patrón que `commander/poe_lift_and_wrist_demo.py`."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        mode, _ = adapter.get_robot_mode()
        if mode == 5:
            return
        time.sleep(0.1)
    print(f"Aviso: RobotMode() no volvió a 5 en {timeout_seconds}s -- leyendo/cerrando igualmente.")


_SNAP_TOLERANCE_DEGREES = 0.5


def _snap_to_exact_start_if_needed(
    configurations: List[JointConfiguration], joint_names: List[str]
) -> None:
    """Añade IN-PLACE la configuración de partida EXACTA
    (`configurations[0]`, la leída del robot real al principio) como
    waypoint final, si la última no coincide ya con ella dentro de
    `_SNAP_TOLERANCE_DEGREES`. Ver docstring/hallazgo (08/09) en
    `commander/cr5_circle_sim_demo.py`: la redundancia de muñeca que
    esquiva la autocolisión puede dejar `joint4`/`joint6` a 1-2° del
    valor de partida, aunque la punta cierre el círculo casi exacta --
    esto lo corrige mandando la partida real tal cual, un movimiento
    pequeño y ya demostrado alcanzable (es de donde salió todo)."""
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
        f"Cierre del círculo a {max_delta:.2f}° de la partida exacta "
        "(redundancia de muñeca) -- añadiendo un waypoint final a la "
        "configuración de partida exacta."
    )
    configurations.append(start)


def main() -> None:
    args = _parse_args()
    joint_names = list(args.joint_names)

    print("=" * 70)
    print("CR5 FÍSICO -- circunferencia COMPLETA antihoraria con la punta, plano Y=cte.")
    print(f"Host: {args.host}   Radio: {args.radius_meters}m   Puntos: {args.num_points}")
    print("Ten una mano cerca del botón físico de parada de emergencia.")
    print("=" * 70)

    kinematics = PoeKinematicsAdapter(steps=1)
    planner = SelfCollisionAvoidingPlanningAdapter(kinematics)
    adapter = Cr5RealRobotAdapter(args.host, joint_names=joint_names)
    try:
        print("\nPaso 1/4 -- consultando el estado del robot (RobotMode)...")
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

        print("\nPaso 2/4 -- leyendo la posición actual (no mueve nada)...")
        start_configuration = adapter.get_current_configuration()
        print(f"Posición actual: {_format_degrees(joint_names, start_configuration)}")

        start_pose = kinematics.forward_kinematics(start_configuration)
        print(
            f"Pose cartesiana de partida: (x={start_pose.x:.4f}, "
            f"y={start_pose.y:.4f}, z={start_pose.z:.4f}). Plano del círculo: "
            f"Y={start_pose.y:.4f} (constante)."
        )

        print("\nResolviendo IK para todos los puntos del círculo (sin mandar nada al robot todavía)...")
        circle_points = _circle_points(start_pose, args.radius_meters, args.num_points)
        configurations = [start_configuration]
        current = start_configuration
        for index, point in enumerate(circle_points[1:], start=1):
            try:
                segment = planner.compute_trajectory(point, current, Scene.empty())
            except SelfCollisionError as error:
                print(
                    f"\nAutocolisión sin rama libre en el punto {index}/{len(circle_points) - 1} "
                    f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
                )
                print("Cancelado -- no se ha mandado NADA al robot todavía.")
                return
            except RuntimeError as error:
                print(
                    f"\nIK no convergió en el punto {index}/{len(circle_points) - 1} "
                    f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
                )
                print("Cancelado -- no se ha mandado NADA al robot todavía.")
                return
            current = segment.waypoints[-1]
            configurations.append(current)

        _snap_to_exact_start_if_needed(configurations, joint_names)

        max_step_degrees = {name: 0.0 for name in joint_names}
        for previous, following in zip(configurations, configurations[1:]):
            for name in joint_names:
                step = abs(
                    math.degrees(following.angle_of(name) - previous.angle_of(name))
                )
                max_step_degrees[name] = max(max_step_degrees[name], step)

        print(
            f"\n{len(configurations)} configuraciones resueltas "
            f"({len(configurations) - 1} MovJ a mandar por la MISMA conexión)."
        )
        print("Paso máximo entre dos waypoints consecutivos, por joint:")
        for name in joint_names:
            print(f"  {name}: {max_step_degrees[name]:.2f}°")
        print(
            f"Configuración final calculada: "
            f"{_format_degrees(joint_names, configurations[-1])}"
        )
        print(
            "\nRecuerda: Cr5RealRobotAdapter valida el límite mecánico absoluto "
            "de cada joint, pero NO valida la velocidad/salto entre waypoints "
            "consecutivos (Bloque 0 #114, sin resolver) -- los pasos de arriba "
            "son pequeños, pero la comprobación de que tiene sentido la haces tú."
        )

        if not _confirm(
            f"\nPaso 3/4 -- ¿confirmas mandar estos {len(configurations) - 1} "
            "MovJ seguidos, ahora mismo (círculo COMPLETO)?"
        ):
            print("Cancelado por el usuario. No se ha mandado ningún movimiento.")
            return

        print("\nPaso 4/4 -- mandando la secuencia...")
        for index, configuration in enumerate(configurations[1:], start=1):
            adapter.set_joints(configuration)
            print(f"  MovJ {index}/{len(configurations) - 1} enviado.")
            time.sleep(args.waypoint_pause_seconds)

        print("\nSecuencia enviada. Esperando a que el robot termine de moverse...")
        _wait_until_robot_idle(adapter)

        print("Releyendo la posición final...")
        after = adapter.get_current_configuration()
        print(f"Posición final real: {_format_degrees(joint_names, after)}")
        final_pose = kinematics.forward_kinematics(after)
        print(
            f"Pose cartesiana final real: (x={final_pose.x:.4f}, "
            f"y={final_pose.y:.4f}, z={final_pose.z:.4f}) -- partida: "
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
