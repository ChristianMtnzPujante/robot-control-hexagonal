"""Coge un cuerpo de la escena y lo deja en un punto con nombre, en una
célula descrita en YAML (`descriptions/cells/`), en modo directo. El mismo script
vale para simulación y para el robot real: solo cambia la célula.

    1. Va a la postura de trabajo (`--posture`), que debe dejar la
       herramienta en la orientación de agarre (p. ej. hacia abajo).
    2. `pick`: se coloca encima del cuerpo, baja en recta, cierra y
       comprueba que lo sujeta; sube.
    3. `place`: lo lleva encima del punto, baja en recta, abre y se retira.

En real, cada bajada pide confirmación por teclado.

Uso:
    ros2 run commander pick_place_demo --cell mesa_cubo --pick cubo --place destino
    ros2 run commander pick_place_demo --cell mi_celda.yaml --pick cubo \\
        --place destino --target real --host 192.168.5.1
"""

from __future__ import annotations

import argparse
import sys

from .cell import InvalidCellError, compile_cell, open_direct
from .manipulation import GraspFailedError, OperationCancelledError


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", required=True, help="Nombre en descriptions/cells/ o ruta a un .yaml.")
    parser.add_argument("--pick", required=True, help="Nombre del cuerpo (scene.bodies) a coger.")
    parser.add_argument("--place", required=True, help="Nombre del punto (scene.points) donde dejarlo.")
    parser.add_argument("--posture", default="pre_agarre", help="Postura de trabajo (por defecto pre_agarre).")
    parser.add_argument("--target", choices=["sim", "real"])
    parser.add_argument("--host")
    args = parser.parse_args()

    cell = compile_cell(args.cell)
    if args.target:
        cell = cell.with_target(args.target, args.host)
    if args.pick not in cell.scene.bodies:
        raise InvalidCellError(f'no hay ningún cuerpo "{args.pick}" en la escena')
    if args.place not in cell.scene.objects:
        raise InvalidCellError(f'no hay ningún punto "{args.place}" en scene.points')
    body = cell.scene.bodies[args.pick]
    if not body.graspable:
        raise InvalidCellError(f'"{args.pick}" no se puede coger (graspable: false en la escena)')
    destination = cell.scene.objects[args.place]

    with open_direct(cell) as handle:
        manipulator = handle.manipulator
        print(f'Postura de trabajo "{args.posture}"...')
        manipulator.move_joints(handle.posture(args.posture))
        try:
            manipulator.pick(args.pick, body)
            manipulator.place(destination)
        except (GraspFailedError, OperationCancelledError) as error:
            print(f"Parado: {error}")
            sys.exit(1)

        if handle.simulation is not None:
            x, y, z = handle.simulation.body_position(args.pick)
            print(
                f'"{args.pick}" en ({x:+.3f}, {y:+.3f}, {z:+.3f}); pedido '
                f"({destination.x:+.3f}, {destination.y:+.3f}, {destination.z:+.3f})."
            )
    print("Hecho.")


if __name__ == "__main__":
    main()
