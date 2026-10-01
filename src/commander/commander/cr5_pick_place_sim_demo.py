"""Prueba de agarre en CoppeliaSim con las herramientas de `manipulation.py`:
el CR5 con la Robotiq 2F-85 coge el cubo de `pick_scene()` y lo deja en otro
punto de la mesa.

Es también el ejemplo de cómo queda un script de prueba: la escena, la
postura de trabajo y dos órdenes (`pick`, `place`). Para el robot real
cambia solo la célula: `real_workcell(host, GraspSettings(grasp_offset=...))`
en vez de `sim_workcell(scene)`, con la escena medida en el laboratorio.

Uso: `ros2 run commander cr5_pick_place_sim_demo` (lanza CoppeliaSim solo si
hace falta). Opcional: --port (por defecto 23000).
"""

from __future__ import annotations

import argparse
import math

from shared_kernel import JointConfiguration, JointPosition, Point

from .cr5_objects_sim_demo import PRE_GRASP_DEGREES, pick_scene
from .manipulation import GraspFailedError
from .workcells import sim_workcell

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Dónde dejar el cubo: el mismo punto de la mesa girado -40° alrededor de la
# base, en la zona libre (lejos de la lata). Misma altura: apoyado.
_PLACE_AT = Point(-0.528, 0.259, 0.025)


def run(port: int = 23000) -> bool:
    scene = pick_scene()
    work_posture = JointConfiguration.create(
        [JointPosition(n, math.radians(d)) for n, d in zip(_JOINT_NAMES, PRE_GRASP_DEGREES)]
    ).value

    with sim_workcell(scene, port=port) as manipulator:
        print("Postura de trabajo (herramienta hacia abajo)...")
        manipulator.move_joints(work_posture)
        try:
            manipulator.pick("cubo", scene.bodies["cubo"])
        except GraspFailedError as error:
            print(f"No ha cogido nada: {error}")
            return False
        manipulator.place(_PLACE_AT)

        # Comprobación contra el simulador: dónde ha quedado de verdad.
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        sim = RemoteAPIClient(port=port).require("sim")
        x, y, z = sim.getObjectPosition(sim.getObject("/cubo"), -1)
        print(f"Cubo en ({x:+.3f}, {y:+.3f}, {z:+.3f}); pedido ({_PLACE_AT.x:+.3f}, {_PLACE_AT.y:+.3f}, {_PLACE_AT.z:+.3f}).")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=23000)
    run(port=parser.parse_args().port)


if __name__ == "__main__":
    main()
