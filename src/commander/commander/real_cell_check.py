"""Prueba CONTROLADA de una célula contra el robot real (y su ensayo en
simulación con el mismo script): las piezas nuevas -- célula compilada,
`Manipulator`, "siempre desde arriba", semillas de IK, `SpeedFactor` -- por
etapas, de menos a más riesgo.

Cada etapa con movimiento sigue el mismo ciclo:
    1. PLANIFICAR en seco (`Manipulator.dry_run`): los waypoints exactos que
       se mandarían, sin mover nada.
    2. REVISAR (`motion_check`): salto máximo entre waypoints, altura mínima
       de la herramienta sobre la mesa. Si algo no cumple, se para.
    3. CONFIRMAR: se enseña el resumen y no se mueve nada sin un "s".
    4. EJECUTAR y MEDIR: se lee dónde ha quedado de verdad y se compara con
       lo planificado. Si se aleja más de lo tolerado, se para.

Etapas (`--stages`, por defecto 0-4):
    0 lectura  -- conecta y lee: articulaciones, brida, modo, pinza. No mueve.
    1 pinza    -- abrir, cerrar y abrir, en el aire. Solo mueve los dedos.
    2 postura  -- va a la postura segura (`--posture`), en articulaciones.
    3 puntos   -- va a cada punto de prueba en el aire (los `prueba*` de la
                  escena) con `move_to_position`, midiendo el error.
    4 recta    -- baja en recta `--linear-drop` y vuelve a subir.
    5 agarre   -- pick + place de un cuerpo. SOLO con `--allow-contact`:
                  es la única etapa que se acerca a la mesa.

La mesa: el plano `mesa` de la escena. Salvo en la etapa 5, ni la brida ni
el punto de agarre bajan de `mesa + --clearance`.

Al salir -- también con Ctrl+C o si algo falla -- se cierra la pinza y se
des-energiza el robot. Ten la seta de emergencia a mano.

Uso:
    # ensayo en simulación, sin preguntas
    ros2 run commander real_cell_check --target sim --yes
    # contra el robot real, a 10 % de velocidad
    ros2 run commander real_cell_check --cell cr5_real_prueba --speed 10
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
from typing import Any, Callable, Dict, List

from shared_kernel import Point, Pose

from .cell import CellDescription, InvalidCellError, compile_cell, open_direct
from .cell.direct import ask_on_keyboard
from .motion_check import MotionLimits, analyze, compare

STAGES = {0: "lectura", 1: "pinza", 2: "postura", 3: "puntos", 4: "recta", 5: "agarre"}
# Tolerancias entre lo planificado y lo medido tras cada movimiento.
_MAX_JOINT_ERROR_DEG = 0.5
_MAX_POSITION_ERROR_MM = 3.0


class Abort(Exception):
    """Algo no cumple: se para la prueba (y se cierra todo)."""


class Check:
    def __init__(self, handle, args, confirm: Callable[[str], bool], table_z: float):
        self.handle = handle
        self.manipulator = handle.manipulator
        self.args = args
        self.confirm = confirm
        self.table_z = table_z
        self.report: List[Dict[str, Any]] = []

    # --- El ciclo de cada movimiento ---------------------------------------------

    def move(self, label: str, action: Callable, min_tool_z: float) -> None:
        manipulator = self.manipulator
        start = manipulator.current_configuration()
        twin, recorder = manipulator.dry_run()
        try:
            action(twin)
        except Exception as error:
            raise Abort(f"{label}: no se puede planificar ({error})") from error
        plan = analyze(start, recorder.waypoints, manipulator.kinematics, manipulator.settings.grasp_offset,
                       MotionLimits(max_step_deg=self.args.max_step, min_tool_z=min_tool_z))
        print(f"\n  [{label}] plan:\n    " + plan.summary().replace("\n", "\n    "))
        entry = {"step": label, "planned": plan.summary().splitlines(), "executed": False}
        self.report.append(entry)
        if not plan.ok:
            raise Abort(f"{label}: el plan no cumple los límites")
        if not self.confirm(f"  [{label}] ¿Ejecutar?"):
            raise Abort(f"{label}: cancelado")
        action(manipulator)
        measured = manipulator.current_configuration()
        joint_error, position_error = compare(recorder.waypoints[-1], measured, manipulator.kinematics)
        entry.update(executed=True, joint_error_deg=round(joint_error, 3), position_error_mm=round(position_error, 2))
        print(f"  [{label}] medido: error articular {joint_error:.2f}° · error de la brida {position_error:.1f} mm")
        if joint_error > _MAX_JOINT_ERROR_DEG or position_error > _MAX_POSITION_ERROR_MM:
            raise Abort(f"{label}: el robot no quedó donde se planificó")

    # --- Etapas ------------------------------------------------------------------

    def stage_reading(self) -> None:
        manipulator = self.manipulator
        configuration = manipulator.current_configuration()
        pose = manipulator.flange_pose()
        joints = {p.joint_name: round(math.degrees(p.angle_radians), 2) for p in configuration.positions}
        print(f"  articulaciones (°): {joints}")
        print(f"  brida: ({pose.x:+.3f}, {pose.y:+.3f}, {pose.z:+.3f})")
        entry: Dict[str, Any] = {"step": "lectura", "joints_deg": joints, "flange": [pose.x, pose.y, pose.z]}
        robot = manipulator.robot
        if hasattr(robot, "get_robot_mode"):
            mode, description = robot.get_robot_mode()
            print(f"  modo del robot: {mode} ({description})")
            entry["robot_mode"] = mode
        if hasattr(robot, "speed_factor"):
            print(f"  velocidad global al habilitar: {robot.speed_factor} %")
        if manipulator.gripper is not None:
            state = manipulator.gripper.get_state()
            print(f"  pinza: apertura {state.opening:.2f}, activada {state.activated}, "
                  f"sujeta {state.holding_object}, fallo 0x{state.fault_code:02X}")
            entry["gripper"] = {"opening": state.opening, "fault": state.fault_code}
        print(f"  mesa a z = {self.table_z:+.3f} m · mínimo para la herramienta {self.min_z:+.3f} m")
        self.report.append(entry)

    def stage_gripper(self) -> None:
        if self.manipulator.gripper is None:
            print("  (sin pinza: se salta)")
            return
        if not self.confirm("  [pinza] Abrir, cerrar y abrir en el aire. ¿Seguir?"):
            raise Abort("pinza: cancelado")
        for name, action in (("abrir", self.manipulator.open), ("cerrar", self.manipulator.close),
                             ("abrir", self.manipulator.open)):
            state = action()
            print(f"  [pinza] {name}: apertura {state.opening:.2f}, sujeta {state.holding_object}")
            self.report.append({"step": f"pinza {name}", "opening": state.opening, "holding": state.holding_object})
            if name == "cerrar" and state.holding_object:
                raise Abort("pinza: dice que sujeta algo al cerrar en el aire")

    def stage_posture(self) -> None:
        target = self.handle.posture(self.args.posture)
        self.move(f"postura {self.args.posture}", lambda m: m.move_joints(target), self.min_z)

    def stage_points(self) -> None:
        points = self.test_points()
        if not points:
            print("  (la escena no tiene puntos 'prueba*': se salta)")
        for name, point in points.items():
            self.move(f"punto {name}", lambda m, p=point: m.move_to_position(p), self.min_z)

    def stage_linear(self) -> None:
        drop = self.args.linear_drop
        start = self.manipulator.flange_pose()
        down = Pose(start.x, start.y, start.z - drop, start.qx, start.qy, start.qz, start.qw)
        self.move(f"recta -{drop * 100:.0f} cm", lambda m: m.move_linear(down), self.min_z)
        self.move(f"recta +{drop * 100:.0f} cm", lambda m: m.move_linear(start), self.min_z)

    def stage_grasp(self) -> None:
        if not self.args.allow_contact:
            print("  (sin --allow-contact: se salta)")
            return
        scene = self.handle.scene
        body, point = self.args.pick, self.args.place
        if body not in scene.bodies or point not in scene.objects:
            raise Abort(f'agarre: hace falta --pick (cuerpos: {", ".join(scene.bodies)}) y '
                        f'--place (puntos: {", ".join(scene.objects)})')
        if self.handle.description.robot.target == "real" and not self.args.yes:
            answer = input("  [agarre] ¿El grasp_offset de la célula está MEDIDO en este montaje? Escribe 'medido': ")
            if answer.strip().lower() != "medido":
                raise Abort("agarre: grasp_offset sin confirmar")
        near_table = self.table_z + 0.005
        self.move(f"pick {body}", lambda m: m.pick(body, scene.bodies[body]), near_table)
        holding = self.manipulator.gripper.get_state().holding_object
        print(f"  [agarre] sujeta: {holding}")
        if not holding:
            raise Abort("agarre: la pinza no detecta el objeto")
        self.move(f"place {point}", lambda m: m.place(scene.objects[point]), near_table)

    # --- Utilidades --------------------------------------------------------------

    @property
    def min_z(self) -> float:
        return self.table_z + self.args.clearance

    def test_points(self) -> Dict[str, Point]:
        objects = self.handle.scene.objects
        if self.args.points:
            missing = [n for n in self.args.points if n not in objects]
            if missing:
                raise Abort(f"puntos que no están en la escena: {missing}")
            return {n: objects[n] for n in self.args.points}
        return {n: p for n, p in objects.items() if n.startswith("prueba")}


def _table_height(cell: CellDescription) -> float:
    plane = cell.scene.planes.get("mesa")
    if plane is None:
        raise InvalidCellError('la escena necesita un plano "mesa" (scene.planes.mesa) para saber cuánto puede bajar')
    if abs(plane.normal.z) < 0.99 * math.sqrt(plane.normal.x ** 2 + plane.normal.y ** 2 + plane.normal.z ** 2):
        raise InvalidCellError('el plano "mesa" tiene que ser horizontal (normal [0, 0, 1])')
    return plane.point.z


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", default="cr5_real_prueba", help="Célula (por defecto cr5_real_prueba).")
    parser.add_argument("--target", choices=["sim", "real"], help="Cambia el destino del YAML.")
    parser.add_argument("--host")
    parser.add_argument("--speed", type=int, default=10, help="Velocidad global en %% en el real (por defecto 10).")
    parser.add_argument("--stages", default="0,1,2,3,4", help="Etapas a ejecutar, p. ej. 0,1,2 (por defecto 0-4).")
    parser.add_argument("--posture", default="prueba_segura", help="Postura segura de la etapa 2.")
    parser.add_argument("--points", type=lambda s: s.split(","), help="Puntos de la etapa 3 (por defecto, prueba*).")
    parser.add_argument("--clearance", type=float, default=0.10, help="Altura mínima sobre la mesa (m), salvo etapa 5.")
    parser.add_argument("--linear-drop", type=float, default=0.05, help="Cuánto baja la etapa 4 (m).")
    parser.add_argument("--max-step", type=float, default=5.0, help="Salto máximo entre waypoints (°).")
    parser.add_argument("--allow-contact", action="store_true", help="Permite la etapa 5 (agarre).")
    parser.add_argument("--pick", help="Cuerpo de la etapa 5.")
    parser.add_argument("--place", help="Punto de la etapa 5.")
    parser.add_argument("--yes", action="store_true", help="Confirma todo solo. SOLO en simulación.")
    parser.add_argument("--report", help="Fichero JSON con el informe (por defecto real_cell_check_<fecha>.json).")
    args = parser.parse_args()

    cell = compile_cell(args.cell)
    if args.target:
        cell = cell.with_target(args.target, args.host)
    if args.yes and cell.robot.target == "real":
        sys.exit("--yes solo se admite en simulación: contra el robot real se confirma cada paso.")
    if cell.robot.target == "real":
        cell = cell.with_speed_factor(args.speed)
    stages = sorted(int(s) for s in args.stages.split(","))
    if 5 in stages and not args.allow_contact:
        sys.exit("La etapa 5 (agarre) necesita --allow-contact.")
    table_z = _table_height(cell)
    confirm = (lambda message: True) if args.yes else ask_on_keyboard

    print(f'Célula "{cell.name}" contra {cell.robot.target.upper()}'
          + (f" ({cell.robot.host}, velocidad {cell.robot.speed_factor} %)" if cell.robot.target == "real" else ""))
    print(f"Etapas: {', '.join(f'{s} {STAGES[s]}' for s in stages)}")
    if cell.robot.target == "real":
        print("Seta de emergencia a mano. Nada se mueve sin confirmar.")

    report: List[Dict[str, Any]] = []
    status = "completa"
    try:
        with open_direct(cell, confirm=confirm) as handle:
            check = Check(handle, args, confirm, table_z)
            try:
                for stage in stages:
                    print(f"\n== Etapa {stage}: {STAGES[stage]}")
                    getattr(check, {0: "stage_reading", 1: "stage_gripper", 2: "stage_posture", 3: "stage_points",
                                    4: "stage_linear", 5: "stage_grasp"}[stage])()
            finally:
                report = check.report
    except Abort as reason:
        status = f"parada: {reason}"
        print(f"\nPARADA: {reason}")
    except KeyboardInterrupt:
        status = "interrumpida (Ctrl+C)"
        print("\nInterrumpida. Robot des-energizado.")
    path = args.report or f"real_cell_check_{datetime.datetime.now():%Y%m%d_%H%M%S}.json"
    with open(path, "w") as file:
        json.dump({"cell": cell.name, "target": cell.robot.target, "speed_factor": cell.robot.speed_factor,
                   "stages": stages, "status": status, "steps": report}, file, ensure_ascii=False, indent=2)
    print(f"\nPrueba {status}. Informe: {path}")
    if status != "completa":
        sys.exit(1)


if __name__ == "__main__":
    main()
