"""Pruebas SENCILLAS de comparación PoE frente a GA (gafro) en CoppeliaSim,
con el cálculo EXPLICADO paso a paso -- la versión mínima de
`cr5_poe_vs_gafro_sim_demo.py`, sin barridos aleatorios ni informes: un
objetivo, dos adaptadores, y por consola en qué coinciden y en qué se
separan los dos cálculos, con los números reales de cada paso. En
CoppeliaSim se ven las dos trayectorias una detrás de otra (rastro azul =
PoE, verde = GA).

Paso 0 (una vez) -- el modelo: los mismos datos del URDF vistos como twist
de se(3) (PoE) y como eje + motor fijo (GA); la FK en la home da la misma
matriz.

Cada prueba recorre la IK en el orden en que se calcula:
  1. Pose actual (FK): matriz 4x4 frente a motor -- misma posición.
  2. Error hacia el objetivo (iteración 0): twist espacial log(T_goal·T⁻¹)
     frente a logaritmo del motor de error log(M_goal·M~). Es AQUÍ donde se
     separan: la parte rotacional coincide siempre; la traslacional solo si
     no hay giro (gafro toma la traslación tal cual, PoE el tornillo).
  3. Jacobiano: el mismo, una vez reordenada la base de bivectores.
  4. Iteraciones de Newton-Raphson lado a lado (distancia de la punta,
     ángulo de error y separación articular entre los dos).
  5. Resultado: iteraciones, tiempo, configuración final, error real de la
     punta en CoppeliaSim.

Prueba 1 -- "bajar 5 cm desde la home". Traslación pura desde una postura
muy singular (brazo estirado en vertical y `joint5 = 0`: el Jacobiano tiene
rango 3 de 6). El error y las cuentas son idénticos en los dos, pero al
salir de la singularidad el redondeo se amplifica y acaban en puntos
distintos de la familia de soluciones: misma pose, articulaciones
distintas, y NO por culpa del álgebra.

Prueba 2 -- "8 cm en -X y 5 cm en -Z desde una postura doblada" (`joint2 =
30°, joint3 = -60°, joint5 = 40°`). Traslación pura y solución aislada: los
dos Newton-Raphson hacen prácticamente las mismas cuentas.

Prueba 3 -- "cambiar de posición Y de orientación" (objetivo = la pose de
`[20, 45, -80, 10, 60, 30]°`, ~46° de giro desde la postura doblada). Aquí
el logaritmo de gafro y el twist de PoE difieren desde la iteración 0 y
cada método hace su propio camino hasta la misma pose.

Uso: `ros2 run commander cr5_poe_vs_gafro_simple_demo` (lanza CoppeliaSim
si hace falta). Opcional: --step-pause-seconds (0.15), --port (23000),
--no-sim (solo los cálculos por consola, sin CoppeliaSim).
"""

from __future__ import annotations

import argparse
import time
from typing import Dict, Optional

import numpy as np

from controller_node.adapters import poe_adapter
from controller_node.adapters.ga_adapter import (
    GaKinematicsAdapter,
    generator_to_twist_order,
)
from controller_node.adapters.poe_adapter import IkIteration, PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, Pose, Scene, Trajectory

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_TRAIL_COLORS = {"poe": [0.0, 0.4, 1.0], "ga": [0.0, 0.8, 0.2]}
# Por debajo de esto dos números "son el mismo" (redondeo de coma flotante).
_SAME = 1e-9


def _configuration(values) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, float(value)) for name, value in zip(_JOINT_NAMES, values)]
    ).value


def _shifted(pose: Pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    """Misma orientación, posición desplazada."""
    return Pose(
        x=pose.x + dx, y=pose.y + dy, z=pose.z + dz,
        qx=pose.qx, qy=pose.qy, qz=pose.qz, qw=pose.qw,
    )


def _angles(configuration: JointConfiguration) -> np.ndarray:
    return np.array([configuration.angle_of(name) for name in _JOINT_NAMES])


def _position(pose: Pose) -> np.ndarray:
    return np.array([pose.x, pose.y, pose.z])


def _vec(values, decimals: int = 4) -> str:
    return "(" + ", ".join(f"{value:+.{decimals}f}" for value in values) + ")"


def _section(title: str) -> None:
    print(f"\n  -- {title} --")


# La demo compara los INTERNOS de los dos adaptadores (twists, motores,
# Jacobianos), así que usa sus funciones privadas a propósito; las trazas
# de la IK sí son públicas (`last_trace`, `last_iteration_count`).


def _explain_model(poe: PoeKinematicsAdapter, ga: GaKinematicsAdapter) -> None:
    print("\n=== Paso 0: el modelo -- mismos datos del URDF, dos álgebras ===")
    print(
        "  Los dos adaptadores parten del MISMO RobotDescription (origen xyz/rpy y eje\n"
        "  de cada articulación). Lo que cambia es cómo se representa cada articulación:"
    )
    index = 1  # joint2: la primera con desplazamiento respecto del eje de la base
    screw = poe._screw_axes[index]
    frame = ga._joints[index].getFrame()
    # El eje como lo monta `_build_system`: URDF (x, y, z) -> [z, -y, x].
    x, y, z = poe_adapter._DEFAULT_CR5_DESCRIPTION.joints[index].axis
    axis_generator = np.array([z, -y, x])
    print(f"\n  joint2 en PoE: twist S2 = (w; v) = {_vec(screw[:3], 3)} ; {_vec(screw[3:], 3)}")
    print(
        "    w = eje de giro en la base, v = -w x q (q = punto del eje, a 0.147 m de\n"
        "    altura): un TORNILLO que ya lleva dentro dónde está el eje."
    )
    print(f"  joint2 en GA:  eje = RotorGenerator [e12, e13, e23] = {_vec(axis_generator, 3)}")
    print(f"                 frame F2 = motor T·R del <origin> = {_vec(np.asarray(frame.vector()).ravel(), 4)}")
    print(
        "    el eje es un bivector en el frame LOCAL de la articulación, y el motor fijo\n"
        "    F2 dice dónde está ese frame: M(θ) = Π F_i·R_i(θ_i) (8 coeficientes por motor)."
    )

    home = np.zeros(6)
    transform = poe_adapter._forward_kinematics(poe._screw_axes, poe._home_pose, home)
    motor = ga._ee_motor(home)
    motor_matrix = np.asarray(motor.toTransformationMatrix())
    print("\n  FK en la home, PoE (e^[S1]θ1 ··· e^[S6]θ6 · M, matriz 4x4):")
    for row in transform[:3]:
        print(f"    {_vec(row, 4)}")
    print(f"  FK en la home, GA (motor, 8 coeficientes): {_vec(np.asarray(motor.vector()).ravel(), 4)}")
    difference = np.abs(transform - motor_matrix).max()
    print(f"  Motor GA pasado a matriz 4x4 vs matriz PoE: diferencia máx {difference:.1e}")
    print("  => misma cadena cinemática; lo que cambia es el álgebra, no el robot.")


def _explain_initial_error(poe_first: IkIteration, ga_first: IkIteration) -> None:
    _section("2. Error hacia el objetivo (iteración 0)")
    print(f"    PoE  twist espacial log(T_goal·T⁻¹)  w = {_vec(poe_first.error[:3])}  v = {_vec(poe_first.error[3:])}")
    print(f"    GA   log(M_goal·M~) (reordenado)    w = {_vec(ga_first.error[:3])}  t = {_vec(ga_first.error[3:])}")
    rotation_gap = np.abs(poe_first.error[:3] - ga_first.error[:3]).max()
    translation_gap = np.abs(poe_first.error[3:] - ga_first.error[3:]).max()
    print(f"    diferencia: parte rotacional {rotation_gap:.1e}, parte traslacional {translation_gap:.1e}")
    angle_deg = np.degrees(poe_first.orientation_angle)
    if translation_gap < _SAME:
        print(
            f"    => IGUALES. El giro que falta es {angle_deg:.4f}°: sin giro, el tornillo de se(3)\n"
            "       es una traslación pura y el logaritmo del motor también. Los dos piden\n"
            "       exactamente el mismo movimiento."
        )
    else:
        print(
            f"    => La rotación coincide (mismo eje y ángulo, {angle_deg:.1f}° de giro que falta),\n"
            "       pero la traslación NO:\n"
            "       - PoE usa v = G(θ)⁻¹·p: la velocidad de un TORNILLO que gira y avanza a la\n"
            "         vez, así que la traslación depende del giro.\n"
            "       - gafro toma t = la traslación del motor de error TAL CUAL, como si girar y\n"
            "         trasladar fueran independientes (log(T·R) ≈ log T + log R).\n"
            "       Coinciden solo a primer orden: cuanto mayor el giro, más se separan. Como el\n"
            "       Jacobiano (paso 3) es el mismo, a partir de aquí cada uno da pasos distintos."
        )


def _explain_jacobian(poe: PoeKinematicsAdapter, ga: GaKinematicsAdapter, thetas: np.ndarray) -> None:
    _section("3. Jacobiano en la configuración de partida")
    jacobian_poe = poe_adapter._jacobian_space(poe._screw_axes, thetas)
    jacobian_ga_raw = ga._geometric_jacobian(thetas)
    jacobian_ga = np.column_stack([generator_to_twist_order(column) for column in jacobian_ga_raw.T])
    print(f"    columna de joint2  PoE (w; v)                = {_vec(jacobian_poe[:, 1])}")
    print(f"    columna de joint2  GA [e12,e13,e23,e1i,e2i,e3i] = {_vec(jacobian_ga_raw[:, 1])}")
    print(f"    GA reordenado a (w; v): e23→wx, -e13→wy, e12→wz = {_vec(jacobian_ga[:, 1])}")
    print(f"    diferencia máx de las 36 entradas tras reordenar: {np.abs(jacobian_poe - jacobian_ga).max():.1e}")
    print(
        "    => MISMO Jacobiano: cada columna es el eje de la articulación movido por las\n"
        "       anteriores (Ad_T·S_i en PoE, M·B_i·M~ en GA). Solo cambia el orden de la base."
    )
    singular_values = np.linalg.svd(jacobian_poe, compute_uv=False)
    # 1e-4 y no 0 exacto: el URDF redondea π/2 a 1.5708, así que las
    # direcciones perdidas dan valores singulares de ~1e-6, no de 1e-16.
    lost = int(np.sum(singular_values < 1e-4))
    print(f"    valores singulares: {_vec(singular_values, 4)}")
    if lost:
        print(
            f"    => SINGULAR: {lost} de 6 direcciones perdidas (valor singular ≈ 0). El brazo no puede\n"
            "       moverse en ellas desde aquí; el amortiguamiento (λ²) evita que el paso se\n"
            "       dispare, pero los primeros pasos apenas avanzan y cuando la postura sale de la\n"
            "       singularidad el paso puede ser enorme. Le pasa igual a los dos: es del robot,\n"
            "       no del álgebra."
        )


def _explain_iterations(poe_trace, ga_trace) -> None:
    _section("4. Newton-Raphson, iteración a iteración")
    print(
        "    Cada paso: Δθ = Jᵀ(J·Jᵀ + λ²I)⁻¹·error, mismo amortiguamiento, mismas tolerancias\n"
        "    (punta 0.1 mm, giro 1 mrad). 'sep.' = máx |θ_PoE - θ_GA| en esa iteración."
    )
    print("     it |        PoE: punta    giro |         GA: punta    giro |  sep. (mrad)")
    for index in range(max(len(poe_trace), len(ga_trace))):
        cells = []
        for trace in (poe_trace, ga_trace):
            if index < len(trace):
                record = trace[index]
                mark = "  ok" if record.step is None else "    "
                cells.append(f"{1e3 * record.tip_distance:10.4f} mm {1e3 * record.orientation_angle:8.3f} mrad{mark}")
            else:
                cells.append(f"{'(ya terminó)':>26}    ")
        if index < len(poe_trace) and index < len(ga_trace):
            separation = f"{1e3 * np.abs(poe_trace[index].thetas - ga_trace[index].thetas).max():10.4f}"
        else:
            separation = f"{'-':>10}"
        print(f"    {index:3d} | {cells[0]} | {cells[1]} | {separation}")

    for name, trace in (("PoE", poe_trace), ("GA", ga_trace)):
        # Solo retrocesos de más de 1 mm: los de décimas son ruido del redondeo.
        worse = [
            (earlier, later) for earlier, later in zip(trace, trace[1:])
            if later.tip_distance - earlier.tip_distance > 1e-3
        ]
        if worse:
            earlier, later = max(worse, key=lambda pair: pair[1].tip_distance - pair[0].tip_distance)
            print(
                f"    => {name}: en la iteración {later.iteration} la punta se ALEJA del objetivo "
                f"({1e3 * earlier.tip_distance:.1f} → {1e3 * later.tip_distance:.1f} mm)."
            )
    if len(poe_trace) != len(ga_trace):
        print(f"    => Iteraciones hasta la tolerancia: PoE {len(poe_trace) - 1}, GA {len(ga_trace) - 1}.")


def _run_case(
    title: str,
    robot,
    poe: PoeKinematicsAdapter,
    ga: GaKinematicsAdapter,
    start: JointConfiguration,
    goal: Pose,
    pause: float,
) -> None:
    print(f"\n=== {title} ===")
    print(f"Objetivo: x={goal.x:.4f} y={goal.y:.4f} z={goal.z:.4f}")

    adapters = {"poe": poe, "ga": ga}
    trajectories: Dict[str, Trajectory] = {}
    elapsed_ms: Dict[str, float] = {}
    for name, adapter in adapters.items():
        started = time.perf_counter()
        trajectories[name] = adapter.compute_trajectory(goal, start)
        elapsed_ms[name] = 1e3 * (time.perf_counter() - started)

    _section("1. Pose actual (FK)")
    poe_start = _position(poe.forward_kinematics(start))
    ga_start = _position(ga.forward_kinematics(start))
    print(f"    PoE (producto de exponenciales, matriz 4x4): p = {_vec(poe_start)}")
    print(f"    GA  (producto de motores F_i·R_i(θ_i))     : p = {_vec(ga_start)}")
    print(f"    diferencia {1e6 * np.linalg.norm(poe_start - ga_start):.2e} µm; "
          f"distancia al objetivo {1e3 * np.linalg.norm(_position(goal) - poe_start):.1f} mm")

    _explain_initial_error(poe.last_trace[0], ga.last_trace[0])
    _explain_jacobian(poe, ga, _angles(start))
    _explain_iterations(poe.last_trace, ga.last_trace)

    _section("5. Resultado")
    finals = {name: trajectory.waypoints[-1] for name, trajectory in trajectories.items()}
    if robot is not None:
        robot.mark_goal(goal)
    for name, adapter in adapters.items():
        real_error = ""
        if robot is not None:
            robot.set_trail_color(_TRAIL_COLORS[name])
            robot.set_joints(start)
            time.sleep(pause)
            for waypoint in trajectories[name].waypoints:
                robot.set_joints(waypoint)
                time.sleep(pause)
            sim_tip = np.array(robot.tip_position())
            real_error = f" | error real de la punta en CoppeliaSim: {1e3 * np.linalg.norm(sim_tip - _position(goal)):.3f} mm"
        print(
            f"    [{name.upper():3}] {adapter.last_iteration_count:>2} iteraciones, "
            f"{elapsed_ms[name]:6.2f} ms{real_error}"
        )
        print(f"          articulaciones finales (°): {np.round(np.degrees(_angles(finals[name])), 3).tolist()}")

    difference = np.abs(_angles(finals["poe"]) - _angles(finals["ga"]))
    print(f"    Diferencia articular final PoE-GA (mrad): {np.round(1e3 * difference, 3).tolist()} "
          f"-> máx {1e3 * difference.max():.3f} mrad ({np.degrees(difference.max()):.3f}°)")
    if np.degrees(difference.max()) < 0.5:
        print("    => Misma pose y la MISMA configuración articular: lo que queda es el margen de la\n"
              "       tolerancia de la IK (0.1 mm / 1 mrad); cada uno para en un punto distinto de él.")
        return
    print("    => Misma pose, DISTINTA configuración articular: hay una familia continua de\n"
          "       soluciones (redundancia) y la IK no fija cuál.")
    same_start = np.abs(poe.last_trace[0].error - ga.last_trace[0].error).max() < _SAME
    if same_start:
        split = next(
            (a.iteration for a, b in zip(poe.last_trace, ga.last_trace)
             if np.abs(a.thetas - b.thetas).max() > 1e-3),
            None,
        )
        print("       Ojo: aquí el álgebra NO es la causa. El error inicial era idéntico y los dos\n"
              f"       hicieron las mismas cuentas hasta la iteración {split - 1}; diferencias de redondeo\n"
              "       (~1e-16) se amplificaron al salir de la singularidad y cada uno cayó en un\n"
              "       punto distinto de la familia de soluciones.")


def run(step_pause_seconds: float, port: int, use_sim: bool) -> None:
    home = _configuration(np.zeros(6))
    robot: Optional[object] = None
    if use_sim:
        ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_poe_vs_gafro_simple_demo_{port}")
        robot = build_cr5_scene(port=port, initial_configuration=home, scene=Scene.empty())

    poe = PoeKinematicsAdapter()
    ga = GaKinematicsAdapter()
    _explain_model(poe, ga)

    # Prueba 1: bajar 5 cm desde la home (con joint5 = 0: muñeca singular).
    _run_case(
        "Prueba 1: bajar 5 cm desde la home (traslación pura, muñeca singular)",
        robot, poe, ga, home, _shifted(poe.forward_kinematics(home), dz=-0.05), step_pause_seconds,
    )

    # Prueba 2: desde una postura doblada (joint5 = 40°, lejos de la
    # singularidad), 8 cm en -X y 5 cm en -Z manteniendo la orientación.
    bent = _configuration(np.radians([0.0, 30.0, -60.0, 0.0, 40.0, 0.0]))
    _run_case(
        "Prueba 2: desde postura doblada, 8 cm en -X y 5 cm en -Z (traslación pura)",
        robot, poe, ga, bent, _shifted(poe.forward_kinematics(bent), dx=-0.08, dz=-0.05),
        step_pause_seconds,
    )

    # Prueba 3: mismo punto de partida, pero el objetivo también GIRA la
    # punta -- la pose de otra configuración conocida, así seguro que es
    # alcanzable.
    rotated_goal = poe.forward_kinematics(_configuration(np.radians([20.0, 45.0, -80.0, 10.0, 60.0, 30.0])))
    _run_case(
        "Prueba 3: desde postura doblada, cambiar posición Y orientación",
        robot, poe, ga, bent, rotated_goal, step_pause_seconds,
    )

    if robot is not None:
        robot.set_trail_color(_TRAIL_COLORS["poe"])
        print("\nListo. Rastro azul = PoE, rastro verde = GA. El robot queda en el final de la prueba 3 (GA).")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--step-pause-seconds", type=float, default=0.15)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    parser.add_argument("--no-sim", action="store_true", help="solo los cálculos, sin CoppeliaSim")
    args = parser.parse_args()
    run(step_pause_seconds=args.step_pause_seconds, port=args.port, use_sim=not args.no_sim)


if __name__ == "__main__":
    main()
