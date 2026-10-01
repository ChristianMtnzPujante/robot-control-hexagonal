"""Abre una célula descrita en YAML (`descriptions/cells/`) en modo directo y, si
se pide, la lleva a una postura con nombre. Sirve para ver una escena nueva
antes de escribir nada más: en simulación la construye entera (robot,
herramienta y cuerpos).

Uso:
    ros2 run commander cell_demo --cell mesa_cubo --posture pre_agarre
    ros2 run commander cell_demo --cell mi_celda.yaml --target real --host 192.168.5.1

    --target/--host cambian el destino que dice el YAML (sim | real).
"""

from __future__ import annotations

import argparse

from .cell import compile_cell, open_direct


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", required=True, help="Nombre en descriptions/cells/ o ruta a un .yaml.")
    parser.add_argument("--posture", help="Postura con nombre a la que ir tras abrir la célula.")
    parser.add_argument("--target", choices=["sim", "real"])
    parser.add_argument("--host")
    args = parser.parse_args()

    cell = compile_cell(args.cell)
    if args.target:
        cell = cell.with_target(args.target, args.host)

    print(f'Célula "{cell.name}": robot {cell.robot.model.name} ({cell.robot.target}), '
          f'herramienta {cell.tool.model.name if cell.tool else "ninguna"}, cinemática {cell.kinematics}')
    for name, body in cell.scene.bodies.items():
        kind = "se puede coger" if body.graspable else "fijo"
        p = body.pose
        print(f"  {name:8s} {type(body.shape).__name__:8s} en ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})  [{kind}]")

    with open_direct(cell) as handle:
        if args.posture:
            print(f'Postura "{args.posture}"...')
            handle.manipulator.move_joints(handle.posture(args.posture))
        p = handle.manipulator.flange_pose()
        print(f"Brida en ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}).")


if __name__ == "__main__":
    main()
