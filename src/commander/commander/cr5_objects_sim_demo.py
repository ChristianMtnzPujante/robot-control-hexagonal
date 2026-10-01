"""El CR5 con la Robotiq 2F-85, en una escena con cuerpos sólidos definidos
en el dominio (`Scene.bodies`): una mesa fija, un cubo y un cilindro que se
pueden coger (`graspable=True`). El brazo va de home a la postura de
PRE-AGARRE, con la pinza abierta y apuntando hacia abajo justo encima del
cubo -- el punto de partida de la prueba de agarre.

Los cuerpos se crean antes de empezar, a partir de `pick_scene()`, con
`build_cr5_scene(..., scene=pick_scene())`: el `scene_builder` dibuja cada
`Body` (ver `_render_bodies`). La escena vive en el dominio; CoppeliaSim
solo la representa.

Uso: `ros2 run commander cr5_objects_sim_demo` (lanza CoppeliaSim solo si
hace falta). Opcional: --port (por defecto 23000).
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List

from shared_kernel import (
    Body,
    Box,
    Cylinder,
    JointConfiguration,
    JointPosition,
    Pose,
    Scene,
    Trajectory,
)

from .coppeliasim_scene_builder import (
    ROBOTIQ_2F_85_ON_CR5,
    build_cr5_scene,
    ensure_coppeliasim_running,
    robotiq_2f_85_gripper,
)

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_STEPS = 50
_STEP_PAUSE_SECONDS = 0.04

# Postura de pre-agarre: joint2 + joint3 + joint4 = 90° deja la brida
# mirando hacia abajo. Según PoeKinematicsAdapter, la brida queda en
# (-0.571, -0.141, 0.265): "delante" del CR5 es -x.
PRE_GRASP_DEGREES = [0.0, 20.0, 100.0, -30.0, -90.0, 0.0]
_FLANGE_XY = (-0.571, -0.141)

# El robot está atornillado a la mesa: su tablero queda en z = 0.
_TABLE_TOP_Z = 0.0
_CUBE_SIZE = 0.05  # 5 cm: cabe de sobra en los 85 mm de la 2F-85


def pick_scene() -> Scene:
    """Mesa + dos cuerpos que se pueden coger. El cubo queda justo debajo
    de la brida en `PRE_GRASP_DEGREES`."""
    table = Body(
        Box(1.1, 1.0, 0.05),
        Pose(-0.35, -0.1, _TABLE_TOP_Z - 0.025),
    )
    cube = Body(
        Box(_CUBE_SIZE, _CUBE_SIZE, _CUBE_SIZE),
        Pose(_FLANGE_XY[0], _FLANGE_XY[1], _TABLE_TOP_Z + _CUBE_SIZE / 2),
        graspable=True,
    )
    can = Body(
        Cylinder(radius=0.025, height=0.10),
        Pose(-0.45, -0.35, _TABLE_TOP_Z + 0.05),
        graspable=True,
        color=(0.15, 0.35, 0.85),
    )
    return (
        Scene.empty()
        .with_body("mesa", table)
        .with_body("cubo", cube)
        .with_body("lata", can)
    )


def _configuration(degrees: List[float]) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, math.radians(value)) for name, value in zip(_JOINT_NAMES, degrees)]
    ).value


def run(port: int = _ZMQ_PORT) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_objects_sim_demo_{port}")

    scene = pick_scene()
    home = _configuration([0.0] * 6)
    robot = build_cr5_scene(
        port=port,
        initial_configuration=home,
        scene=scene,
        mounts=[ROBOTIQ_2F_85_ON_CR5],
    )
    gripper = robotiq_2f_85_gripper(port)

    print("Cuerpos en la escena:")
    for name, body in scene.bodies.items():
        kind = "se puede coger" if body.graspable else "fijo"
        print(
            f"  {name:5s} {type(body.shape).__name__:8s} en "
            f"({body.pose.x:+.3f}, {body.pose.y:+.3f}, {body.pose.z:+.3f})  [{kind}]"
        )

    print("Pinza: abriendo...")
    gripper.set_opening(0.0)

    pre_grasp = _configuration(PRE_GRASP_DEGREES)
    print("Brazo: de home a la postura de pre-agarre, sobre el cubo...")
    for waypoint in Trajectory.straight_line(home, pre_grasp, _STEPS).waypoints:
        robot.set_joints(waypoint)
        time.sleep(_STEP_PAUSE_SECONDS)

    print(
        "Listo: pinza abierta sobre el cubo. Siguiente paso, la prueba de "
        "agarre (bajar, cerrar y subir con el cubo)."
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    run(port=parser.parse_args().port)


if __name__ == "__main__":
    main()
