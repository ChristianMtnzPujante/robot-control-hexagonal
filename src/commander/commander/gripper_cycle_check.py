"""Prueba mínima de la pinza: abrir y cerrar cada `--period` segundos (3 por
defecto), en el aire, hasta Ctrl+C o hasta `--cycles` ciclos. No mueve el
brazo: solo los dedos.

Cada línea enseña la apertura medida (0 abierta .. 1 cerrada) y si la pinza
dice que sujeta algo. Al salir -- también con Ctrl+C -- se cierra la
conexión y se des-energiza el robot (lo hace `open_direct`).

Uso:
    ros2 run commander gripper_cycle_check                  # robot real, sin fin
    ros2 run commander gripper_cycle_check --cycles 5
    ros2 run commander gripper_cycle_check --target sim --yes
"""

from __future__ import annotations

import argparse
import sys
import time

from .cell import compile_cell, open_direct
from .cell.direct import ask_on_keyboard


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", default="cr5_real_prueba", help="Célula (por defecto cr5_real_prueba).")
    parser.add_argument("--target", choices=["sim", "real"], help="Cambia el destino del YAML.")
    parser.add_argument("--host")
    parser.add_argument("--period", type=float, default=3.0, help="Segundos entre órdenes (por defecto 3).")
    parser.add_argument("--cycles", type=int, default=0, help="Ciclos abrir+cerrar (0 = hasta Ctrl+C).")
    parser.add_argument("--yes", action="store_true", help="No pregunta antes de empezar.")
    args = parser.parse_args()

    cell = compile_cell(args.cell)
    if args.target:
        cell = cell.with_target(args.target, args.host)
    print(f'Célula "{cell.name}" contra {cell.robot.target.upper()}'
          + (f" ({cell.robot.host})" if cell.robot.target == "real" else ""))

    try:
        with open_direct(cell) as handle:
            manipulator = handle.manipulator
            if manipulator.gripper is None:
                sys.exit("Esta célula no tiene pinza.")
            if not args.yes and not ask_on_keyboard(
                f"Abrir/cerrar la pinza cada {args.period:g} s ({args.cycles or 'sin fin'} ciclos). ¿Seguir?"
            ):
                return
            actions = (("abrir", manipulator.open), ("cerrar", manipulator.close))
            step = 0
            next_time = time.monotonic()
            while not args.cycles or step < 2 * args.cycles:
                name, action = actions[step % 2]
                started = time.monotonic()
                state = action()
                print(f"[{step // 2 + 1}] {name}: apertura {state.opening:.2f}, sujeta {state.holding_object}, "
                      f"fallo {state.fault_code}, {time.monotonic() - started:.2f} s")
                step += 1
                next_time += args.period
                time.sleep(max(0.0, next_time - time.monotonic()))
    except KeyboardInterrupt:
        print("\nInterrumpida.")


if __name__ == "__main__":
    main()
