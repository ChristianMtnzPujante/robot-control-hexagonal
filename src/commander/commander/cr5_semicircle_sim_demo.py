"""Traza con la punta (tip) del CR5, en CoppeliaSim, una semicircunferencia
en sentido ANTIHORARIO, en un plano perpendicular al eje Y (es decir, un
plano Y=constante -- el plano XZ) partiendo de la propia configuración
inicial (home, los 6 joints a 0°).

Geometría (todo en el marco de `PoeKinematicsAdapter`, relativo a
`base_link`, que coincide con el marco mundo de CoppeliaSim -- ver
`coppeliasim_scene_builder.py`):

  - `home_pose = PoeKinematicsAdapter().forward_kinematics(HOME)` da el
    punto de partida real (x0, y0, z0) -- no un valor hardcodeado a mano.
  - El centro de la circunferencia se coloca en (x0, y0, z0 - radius) --
    DIRECTAMENTE DEBAJO de la home, no al lado. La home queda así en el
    ángulo 90° (theta=90°) de la parametrización estándar
    (x = centro_x + r·cos(theta), z = centro_z + r·sin(theta)).
  - Antihorario == theta CRECIENTE (convención matemática estándar) -- se
    recorre theta de 90° a 270°, pasando por 180° (el lado -X). El
    resultado es un arco que baja desde la home, se desvía hacia -X hasta
    la mitad del recorrido, y vuelve a cerrar justo debajo de la home (a
    2·radius de profundidad en Z, mismo X/Y que la home).
  - **Por qué "hacia abajo" y no "hacia arriba" u "hacia el lado", pese a
    partir siempre de la home:** verificado empíricamente (barrido de
    IK real en un grid alrededor de la home, ver Diario 08/09) que con la
    orientación de la home, el CR5 NO puede alcanzar NINGÚN punto con Z
    mayor que el de home, ni ningún punto a la MISMA altura que home salvo
    la propia home -- la home está en el límite superior/lateral de lo
    alcanzable con esa orientación fija, solo hay margen hacia abajo. Un
    primer diseño (cúpula hacia arriba) fallaba en el primer punto del
    arco; otro (semicírculo con ambos extremos a la misma altura) fallaba
    cerca del final, al intentar volver a la altura de home lejos de ella.
    Ver [[PoeKinematicsAdapter]] para la singularidad ya documentada en
    joint5≈0 -- esto es una limitación de alcance distinta, no la misma.
  - La orientación del tip se mantiene FIJA durante todo el recorrido, la
    misma que en la home -- este demo mueve la punta en un arco, no gira
    la muñeca.

Cada punto del arco se resuelve con IK real (`PoeKinematicsAdapter`, no
interpolación en espacio de articulaciones entre dos únicos extremos --
eso NO trazaría un arco) -- `steps=1` en el constructor hace que cada
`compute_trajectory` devuelva solo [origen, IK resuelta], sin
interpolación de más. Se encadenan los resultados (cada punto parte de la
configuración articular del punto anterior) para que la secuencia final
sea continua.

**Autocolisión (08/09):** este arco concreto (radio 0.15m, 10 puntos)
hace que la muñeca se acerque demasiado al antebrazo hacia el final del
recorrido -- disparó una alarma real del CR5 físico (`GetErrorID()`=[76])
la primera vez que se probó. En vez de `PoeKinematicsAdapter` a secas, el
`KinematicsPort` que resuelve cada punto es un
`SelfCollisionAvoidingPlanningAdapter` (ver ese módulo) que, si el punto
directo colisionaría, reintenta la IK desde una muñeca ligeramente
desplazada (explotando la singularidad de `joint5≈0`) hasta encontrar una
rama libre que llegue al MISMO punto cartesiano -- mismos puntos exactos
del arco, trayectoria articular distinta donde hace falta.

Uso: `ros2 run commander cr5_semicircle_sim_demo` (lanza CoppeliaSim solo
si hace falta). En CoppeliaSim: el rastro de dummies azules debería
dibujar una semicircunferencia limpia, empezando en la postura home,
bajando y desviándose hacia un lado, y terminando justo debajo del punto
de partida, sin desviarse en Y.

    Opcional: --radius-meters (por defecto 0.15), --num-points (por
    defecto 10, cuantos más, más fino el arco), --step-pause-seconds (por
    defecto 0.1), --port (por defecto 23000).
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
_DEFAULT_NUM_POINTS = 10
_DEFAULT_STEP_PAUSE_SECONDS = 0.1


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius-meters", type=float, default=_DEFAULT_RADIUS_METERS)
    parser.add_argument("--num-points", type=int, default=_DEFAULT_NUM_POINTS)
    parser.add_argument("--step-pause-seconds", type=float, default=_DEFAULT_STEP_PAUSE_SECONDS)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    return parser.parse_args()


def _semicircle_points(home_pose: Pose, radius: float, num_points: int) -> List[Pose]:
    """Puntos de la home_pose (theta=90°) a theta=270°, ambos incluidos,
    antihorario (theta creciente) en el plano Y=home_pose.y, con el centro
    de la circunferencia DIRECTAMENTE DEBAJO de la home (ver docstring del
    módulo sobre por qué -- la home no puede subir ni mantenerse a su
    misma altura en ningún otro punto, solo bajar). Orientación fija = la
    de home_pose en todos los puntos."""
    center_x = home_pose.x
    center_z = home_pose.z - radius
    points = []
    for i in range(num_points):
        theta = math.pi / 2 + math.pi * i / (num_points - 1)
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


def run(
    radius: float = _DEFAULT_RADIUS_METERS,
    num_points: int = _DEFAULT_NUM_POINTS,
    step_pause_seconds: float = _DEFAULT_STEP_PAUSE_SECONDS,
    port: int = _ZMQ_PORT,
) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_semicircle_sim_demo_{port}")
    robot = build_cr5_scene(port=port, initial_configuration=_HOME, scene=Scene.empty())

    kinematics = PoeKinematicsAdapter(steps=1)
    planner = SelfCollisionAvoidingPlanningAdapter(kinematics)
    home_pose = kinematics.forward_kinematics(_HOME)
    print(
        f"Home: (x={home_pose.x:.4f}, y={home_pose.y:.4f}, z={home_pose.z:.4f}). "
        f"Plano del arco: Y={home_pose.y:.4f} (constante). Radio: {radius}m."
    )

    arc_points = _semicircle_points(home_pose, radius, num_points)
    robot.mark_goal(arc_points[-1])

    print(f"Resolviendo IK para {len(arc_points)} puntos del arco...")
    configurations = [_HOME]
    current = _HOME
    # Se salta el punto 0 (theta=0): es exactamente home_pose, ya cubierto
    # por _HOME como punto de partida -- resolverlo de nuevo sería
    # redundante.
    for index, point in enumerate(arc_points[1:], start=1):
        try:
            segment = planner.compute_trajectory(point, current, Scene.empty())
        except SelfCollisionError as error:
            print(
                f"Autocolisión sin rama libre en el punto {index}/{len(arc_points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el arco aquí -- se anima lo ya resuelto.")
            break
        except RuntimeError as error:
            print(
                f"IK no convergió en el punto {index}/{len(arc_points) - 1} "
                f"(x={point.x:.4f}, z={point.z:.4f}): {error}"
            )
            print("Deteniendo el arco aquí -- se anima lo ya resuelto.")
            break
        current = segment.waypoints[-1]
        configurations.append(current)

    print(f"{len(configurations)} configuraciones resueltas. Animando en CoppeliaSim...")
    for configuration in configurations:
        robot.set_joints(configuration)
        time.sleep(step_pause_seconds)

    final_pose = kinematics.forward_kinematics(configurations[-1])
    print(
        f"Listo. Punto final alcanzado: (x={final_pose.x:.4f}, "
        f"y={final_pose.y:.4f}, z={final_pose.z:.4f})."
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
