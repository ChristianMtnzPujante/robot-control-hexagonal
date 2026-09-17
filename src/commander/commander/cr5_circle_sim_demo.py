"""Traza con la punta (tip) del CR5, en CoppeliaSim, una CIRCUNFERENCIA
COMPLETA (360°, no solo el semicírculo de `cr5_semicircle_sim_demo.py`) en
sentido antihorario, en el plano Y=constante, partiendo y cerrando de
vuelta sobre la propia home.

Misma geometría de base que `cr5_semicircle_sim_demo.py` (centro de la
circunferencia DIRECTAMENTE DEBAJO de la home, home en theta=90°) --
aquí, en vez de barrer solo 90°→270°, se barre 90°→450° (=90°+360°), la
vuelta entera. Por construcción, z(theta) = center_z + radius·sin(theta)
nunca supera `home_z` (sin(theta) ≤ 1 siempre, con igualdad solo en
theta=90°, la propia home) -- así que la circunferencia completa entera
se queda dentro de la misma región alcanzable ya validada para el
semicírculo (ver [[PoeKinematicsAdapter]]/vault, "la home no puede subir,
solo bajar"), sin necesitar ningún caso especial para el tramo de vuelta.

Autocolisión: igual que el semicírculo, este círculo hace que la muñeca
se acerque al antebrazo en varios tramos del recorrido -- el
`KinematicsPort` que resuelve cada punto es un
`SelfCollisionAvoidingPlanningAdapter` (no `PoeKinematicsAdapter` a
secas) desde el principio, no como corrección a posteriori. Verificado en
Python puro antes de tocar CoppeliaSim: los 18 pasos convergen sin
ninguna excepción, cerrando sobre la home con un residuo de <0.1mm de
posición -- pero (encontrado en la primera prueba real, 08/09) `joint4`/
`joint6` individuales pueden quedar a 1-2° del cero exacto, por la misma
redundancia de muñeca que se explota para esquivar la autocolisión
(varias combinaciones de esos dos joints dan la MISMA pose del tip). El
brazo no se veía del todo "recto" al final pese a que la punta sí estaba
exactamente en su sitio -- `_snap_to_exact_start_if_needed` añade un
último waypoint a la configuración de partida EXACTA cuando hace falta.

Cada punto se resuelve con IK real, encadenando cada solución como punto
de partida de la siguiente (mismo mecanismo que el semicírculo).

Uso: `ros2 run commander cr5_circle_sim_demo` (lanza CoppeliaSim solo si
hace falta). En CoppeliaSim: el rastro de dummies azules debería dibujar
una circunferencia completa, bajando desde la home, dando la vuelta entera
por debajo, y cerrando de nuevo sobre el punto de partida.

    Opcional: --radius-meters (por defecto 0.15), --num-points (por
    defecto 19 -- 18 pasos de 20° cada uno, misma resolución angular que
    el semicírculo), --step-pause-seconds (por defecto 0.1), --port (por
    defecto 23000).
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

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_HOME = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value
_DEFAULT_RADIUS_METERS = 0.15
_DEFAULT_NUM_POINTS = 19
_DEFAULT_STEP_PAUSE_SECONDS = 0.1


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius-meters", type=float, default=_DEFAULT_RADIUS_METERS)
    parser.add_argument("--num-points", type=int, default=_DEFAULT_NUM_POINTS)
    parser.add_argument("--step-pause-seconds", type=float, default=_DEFAULT_STEP_PAUSE_SECONDS)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    return parser.parse_args()


def _circle_points(home_pose: Pose, radius: float, num_points: int) -> List[Pose]:
    """Puntos de la home_pose (theta=90°) a theta=450°=90° (vuelta
    completa), ambos incluidos, antihorario, en el plano Y=home_pose.y,
    centro de la circunferencia DIRECTAMENTE DEBAJO de la home -- ver
    docstring del módulo. Orientación fija = la de home_pose en todos los
    puntos."""
    center_x = home_pose.x
    center_z = home_pose.z - radius
    points = []
    for i in range(num_points):
        theta = math.pi / 2 + 2 * math.pi * i / (num_points - 1)
        points.append(
            Pose(
                x=center_x + radius * math.cos(theta),
                y=home_pose.y,
                z=center_z + radius * math.sin(theta),
                qx=home_pose.qx,
                qy=home_pose.qy,
                qz=home_pose.qz,
                qw=home_pose.qw,
            )
        )
    return points


_SNAP_TOLERANCE_DEGREES = 0.5


def _snap_to_exact_start_if_needed(configurations: List[JointConfiguration]) -> None:
    """Añade IN-PLACE la configuración de partida EXACTA
    (`configurations[0]`) como waypoint final, si la última no coincide ya
    con ella dentro de `_SNAP_TOLERANCE_DEGREES`.

    Por qué hace falta: `SelfCollisionAvoidingPlanningAdapter` explota la
    redundancia de `joint4`/`joint6` cerca de `joint5≈0` para esquivar
    autocolisiones -- distintas combinaciones de esos dos joints dan la
    MISMA pose cartesiana del tip. Tras varios "nudges" a lo largo de la
    vuelta completa, la punta cierra el círculo con submilímetro de error
    (ver Diario 08/09), pero `joint4`/`joint6` individuales pueden quedar
    a 1-2° del cero exacto -- el brazo no vuelve visualmente "recto" del
    todo aunque la punta sí esté en el sitio correcto. Como la
    configuración de partida YA demostró ser alcanzable (es de donde
    salió todo el recorrido), mandarla de nuevo tal cual es seguro y
    barato -- un movimiento pequeño, no un salto grande."""
    if not configurations:
        return
    start, last = configurations[0], configurations[-1]
    joint_names = [p.joint_name for p in start.positions]
    max_delta = max(
        abs(math.degrees(last.angle_of(name) - start.angle_of(name)))
        for name in joint_names
    )
    if max_delta <= _SNAP_TOLERANCE_DEGREES:
        return
    print(
        f"Cierre del círculo a {max_delta:.2f}° del cero exacto (redundancia de "
        "muñeca, ver docstring) -- añadiendo un waypoint final a la "
        "configuración de partida exacta."
    )
    configurations.append(start)


def run(
    radius: float = _DEFAULT_RADIUS_METERS,
    num_points: int = _DEFAULT_NUM_POINTS,
    step_pause_seconds: float = _DEFAULT_STEP_PAUSE_SECONDS,
    port: int = _ZMQ_PORT,
) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_circle_sim_demo_{port}")
    robot = build_cr5_scene(port=port, initial_configuration=_HOME, scene=Scene.empty())

    kinematics = PoeKinematicsAdapter(steps=1)
    planner = SelfCollisionAvoidingPlanningAdapter(kinematics)
    home_pose = kinematics.forward_kinematics(_HOME)
    print(
        f"Home: (x={home_pose.x:.4f}, y={home_pose.y:.4f}, z={home_pose.z:.4f}). "
        f"Plano del círculo: Y={home_pose.y:.4f} (constante). Radio: {radius}m."
    )

    circle_points = _circle_points(home_pose, radius, num_points)
    robot.mark_goal(circle_points[-1])

    print(f"Resolviendo IK para {len(circle_points)} puntos del círculo...")
    configurations = [_HOME]
    current = _HOME
    for index, point in enumerate(circle_points[1:], start=1):
        try:
            segment = planner.compute_trajectory(point, current, Scene.empty())
        except SelfCollisionError as error:
            print(
                f"Autocolisión sin rama libre en el punto {index}/{len(circle_points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el círculo aquí -- se anima lo ya resuelto.")
            break
        except RuntimeError as error:
            print(
                f"IK no convergió en el punto {index}/{len(circle_points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el círculo aquí -- se anima lo ya resuelto.")
            break
        current = segment.waypoints[-1]
        configurations.append(current)

    _snap_to_exact_start_if_needed(configurations)

    print(f"{len(configurations)} configuraciones resueltas. Animando en CoppeliaSim...")
    for configuration in configurations:
        robot.set_joints(configuration)
        time.sleep(step_pause_seconds)

    final_pose = kinematics.forward_kinematics(configurations[-1])
    print(
        f"Listo. Punto final alcanzado: (x={final_pose.x:.4f}, "
        f"y={final_pose.y:.4f}, z={final_pose.z:.4f}) -- home real: "
        f"(x={home_pose.x:.4f}, y={home_pose.y:.4f}, z={home_pose.z:.4f})."
    )


def main() -> None:
    args = _parse_args()
    run(
        radius=args.radius_meters,
        num_points=args.num_points,
        step_pause_seconds=args.step_pause_seconds,
        port=args.port,
    )


if __name__ == "__main__":
    main()
