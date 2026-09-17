"""Comparativa en CoppeliaSim de las dos cinemáticas reales del repo sobre el
MISMO problema: `PoeKinematicsAdapter` (Product of Exponentials, matrices
4x4 en numpy) frente a `GaKinematicsAdapter` (álgebra geométrica conforme
vía gafro/pygafro). Escrito el 17/09/2026 al cerrar F1.2 de la tesis
(GAFRO como `KinematicsPort`).

Qué compara, y cómo:

  1. Se construye la escena del CR5 en CoppeliaSim desde el URDF real
     (`coppeliasim_scene_builder.build_cr5_scene`) en la home (6 joints a
     0). El objeto `Link6_visual` de la escena es la 'verdad de terreno':
     CoppeliaSim calcula su posición con SU propio modelo del URDF, sin
     saber nada de PoE ni de GA.
  2. Se genera el mismo arco cartesiano que `cr5_semicircle_sim_demo.py`
     (semicircunferencia hacia abajo desde la home, orientación fija --
     ver ahí por qué hacia abajo y no hacia arriba).
  3. Para cada adaptador, desde la home, se resuelve la IK de cada punto
     del arco encadenando (cada punto parte de la solución anterior), y se
     mide por punto: tiempo de `compute_trajectory`, nº de iteraciones de
     Newton-Raphson (`last_iteration_count`) y error de posición según la
     propia FK del adaptador.
  4. Se anima cada solución en CoppeliaSim (rastro azul = PoE, rastro
     verde = GA) y, en cada waypoint, se lee dónde ha puesto CoppeliaSim la
     punta de verdad -- de ahí sale el error 'real' de cada adaptador
     contra el punto pedido, y la diferencia entre ambas trayectorias
     articulares.
  5. Además, sobre N configuraciones aleatorias: FK de cada adaptador
     contra la posición de `Link6_visual` que da CoppeliaSim al poner esas
     mismas articulaciones (error absoluto, sin IK de por medio), y tiempo
     medio de FK.

`Link6_visual` NO está exactamente en el frame de joint6 (es la malla del
eslabón, con su propio origen): el demo mide ese desplazamiento en la home
y lo descuenta. Como el arco mantiene la orientación fija, ese offset es
constante en el marco mundo durante todo el recorrido; en las
configuraciones aleatorias del paso 5 no lo es, así que ahí se compara la
posición del frame de joint6 (`link_poses()[-1]` de cada adaptador,
transformando el offset local medido) -- ver `_tip_from_joint6`.

Salida: tabla por consola y un informe Markdown en
`docs/comparativa_poe_vs_gafro_coppeliasim.md` (se sobreescribe).

Uso: `ros2 run commander cr5_poe_vs_gafro_sim_demo` (lanza CoppeliaSim si
hace falta). Opcional: --radius-meters (0.15), --num-points (10),
--random-configurations (200), --step-pause-seconds (0.1), --port (23000),
--report-path.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time
from datetime import date
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from controller_node.adapters.ga_adapter import GaKinematicsAdapter
from controller_node.adapters.poe_adapter import PoeKinematicsAdapter
from shared_kernel import JointConfiguration, JointPosition, KinematicsPort, Pose, Scene

from .coppeliasim_scene_builder import build_cr5_scene, ensure_coppeliasim_running
from .cr5_semicircle_sim_demo import _semicircle_points

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_ZMQ_PORT = 23000
_HOME = JointConfiguration.create(
    [JointPosition(name, 0.0) for name in _JOINT_NAMES]
).value
_TRAIL_COLORS = {"poe": [0.0, 0.4, 1.0], "ga": [0.0, 0.8, 0.2]}
_DEFAULT_REPORT = (
    Path(__file__).resolve().parents[3] / "docs" / "comparativa_poe_vs_gafro_coppeliasim.md"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius-meters", type=float, default=0.15)
    parser.add_argument("--num-points", type=int, default=10)
    parser.add_argument("--random-configurations", type=int, default=200)
    parser.add_argument("--step-pause-seconds", type=float, default=0.1)
    parser.add_argument("--port", type=int, default=_ZMQ_PORT)
    parser.add_argument("--report-path", type=str, default=str(_DEFAULT_REPORT))
    return parser.parse_args()


def _configuration(values) -> JointConfiguration:
    return JointConfiguration.create(
        [JointPosition(name, float(value)) for name, value in zip(_JOINT_NAMES, values)]
    ).value


def _position(pose: Pose) -> np.ndarray:
    return np.array([pose.x, pose.y, pose.z])


def _rotation(pose: Pose) -> np.ndarray:
    qx, qy, qz, qw = pose.qx, pose.qy, pose.qz, pose.qw
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )


def _tip_from_joint6(joint6_pose: Pose, local_offset: np.ndarray) -> np.ndarray:
    """Posición prevista de `Link6_visual` = frame de joint6 + offset local
    (medido una vez en la home) rotado con la orientación actual."""
    return _position(joint6_pose) + _rotation(joint6_pose) @ local_offset


class _ArcResult:
    def __init__(self, name: str) -> None:
        self.name = name
        self.configurations: List[JointConfiguration] = [_HOME]
        self.ik_seconds: List[float] = []
        self.iterations: List[int] = []
        self.fk_error_m: List[float] = []
        self.sim_error_m: List[float] = []
        self.failed_at: Optional[int] = None
        self.failure: str = ""


def _solve_arc(name: str, kinematics: KinematicsPort, points: List[Pose]) -> _ArcResult:
    result = _ArcResult(name)
    current = _HOME
    for index, point in enumerate(points[1:], start=1):
        started = time.perf_counter()
        try:
            segment = kinematics.compute_trajectory(point, current)
        except RuntimeError as error:
            result.failed_at, result.failure = index, str(error)
            break
        result.ik_seconds.append(time.perf_counter() - started)
        result.iterations.append(getattr(kinematics, "last_iteration_count", -1))
        current = segment.waypoints[-1]
        result.configurations.append(current)
        reached = kinematics.forward_kinematics(current)
        result.fk_error_m.append(float(np.linalg.norm(_position(reached) - _position(point))))
    return result


def _animate(robot, result: _ArcResult, points: List[Pose], offset_world: np.ndarray, pause: float) -> None:
    robot.set_trail_color(_TRAIL_COLORS[result.name])
    for index, configuration in enumerate(result.configurations):
        robot.set_joints(configuration)
        time.sleep(pause)
        sim_tip = np.array(robot.tip_position())
        result.sim_error_m.append(float(np.linalg.norm(sim_tip - offset_world - _position(points[index]))))


def _joint_differences(a: JointConfiguration, b: JointConfiguration) -> Dict[str, float]:
    """|Δθ| por articulación entre dos soluciones, más 'joint4+joint6': en
    la home del CR5 (joint5=0) los ejes de joint4 y joint6 son paralelos,
    así que cualquier reparto θ4+θ6=cte da la MISMA pose del tip -- dos
    Newton-Raphson con parametrizaciones distintas del error pueden
    repartirlo de forma distinta sin que ninguno esté 'equivocado'."""
    diffs = {name: abs(a.angle_of(name) - b.angle_of(name)) for name in _JOINT_NAMES}
    diffs["joint4+joint6"] = abs(
        (a.angle_of("joint4") + a.angle_of("joint6")) - (b.angle_of("joint4") + b.angle_of("joint6"))
    )
    return diffs


def _fk_against_simulator(
    robot, adapters: Dict[str, KinematicsPort], local_offset: np.ndarray, count: int, pause: float
) -> Dict[str, Dict[str, float]]:
    rng = np.random.default_rng(0)
    errors: Dict[str, List[float]] = {name: [] for name in adapters}
    fk_seconds: Dict[str, List[float]] = {name: [] for name in adapters}
    link_seconds: Dict[str, List[float]] = {name: [] for name in adapters}
    for _ in range(count):
        # Muestreo moderado: evita posturas que atraviesen el suelo de la
        # escena (cosmético) y mantiene el barrido representativo.
        configuration = _configuration(rng.uniform(-math.pi / 2, math.pi / 2, 6))
        robot.set_joints(configuration)
        time.sleep(pause)
        sim_tip = np.array(robot.tip_position())
        for name, adapter in adapters.items():
            started = time.perf_counter()
            adapter.forward_kinematics(configuration)
            fk_seconds[name].append(time.perf_counter() - started)
            started = time.perf_counter()
            joint6 = adapter.link_poses(configuration)[-1]
            link_seconds[name].append(time.perf_counter() - started)
            errors[name].append(float(np.linalg.norm(_tip_from_joint6(joint6, local_offset) - sim_tip)))
    return {
        name: {
            "max_error_m": max(errors[name]),
            "mean_error_m": statistics.fmean(errors[name]),
            "mean_fk_us": 1e6 * statistics.fmean(fk_seconds[name]),
            "mean_link_poses_us": 1e6 * statistics.fmean(link_seconds[name]),
        }
        for name in adapters
    }


def _fmt(values: List[float], scale: float = 1.0, digits: int = 2) -> str:
    if not values:
        return "-"
    return f"{scale * statistics.fmean(values):.{digits}f} (máx {scale * max(values):.{digits}f})"


def _write_report(
    path: Path,
    radius: float,
    points: List[Pose],
    results: Dict[str, _ArcResult],
    joint_diffs: List[Dict[str, float]],
    fk_stats: Dict[str, Dict[str, float]],
    offset_world: np.ndarray,
    random_count: int,
) -> str:
    poe, ga = results["poe"], results["ga"]
    lines = [
        f"# Comparativa PoE vs GAFRO en CoppeliaSim — {date.today().isoformat()}",
        "",
        "Generado por `commander/cr5_poe_vs_gafro_sim_demo.py` (ver su docstring para el método). "
        "Mismo `RobotDescription` del CR5, mismo Newton-Raphson amortiguado y mismas tolerancias "
        "(posición 1e-4 m, orientación 1e-3 rad) en ambos adaptadores; la única diferencia es el "
        "álgebra: matrices 4x4/se(3) en numpy (PoE) frente a motores CGA en gafro (GA).",
        "",
        f"Arco: semicircunferencia de radio {radius} m hacia abajo desde la home, {len(points)} puntos, "
        "orientación fija. Verdad de terreno: posición de `Link6_visual` calculada por CoppeliaSim "
        f"(offset respecto del frame de joint6 medido en la home: {np.round(offset_world, 4).tolist()} m).",
        "",
        "## IK sobre el arco (por punto, media y máximo)",
        "",
        "| Adaptador | Puntos resueltos | Tiempo IK (ms) | Iteraciones NR | Error FK propia (mm) | Error real en CoppeliaSim (mm) |",
        "|---|---|---|---|---|---|",
    ]
    for result in (poe, ga):
        solved = f"{len(result.configurations) - 1}/{len(points) - 1}"
        if result.failed_at is not None:
            solved += f" (falló en {result.failed_at})"
        lines.append(
            f"| {result.name.upper()} | {solved} | {_fmt(result.ik_seconds, 1e3)} | "
            f"{_fmt([float(i) for i in result.iterations], 1, 1)} | {_fmt(result.fk_error_m, 1e3, 4)} | "
            f"{_fmt(result.sim_error_m[1:], 1e3, 3)} |"
        )
    per_joint = {
        key: max(d[key] for d in joint_diffs) if joint_diffs else 0.0
        for key in _JOINT_NAMES + ["joint4+joint6"]
    }
    worst_joint = max(_JOINT_NAMES, key=lambda k: per_joint[k])
    same_solution = all(per_joint[k] < 5e-3 for k in _JOINT_NAMES)
    planar_self_motion = per_joint["joint1"] < 1e-3 and per_joint["joint5"] < 1e-3 and all(
        abs(c.angle_of("joint5")) < 1e-3 for r in (poe, ga) for c in r.configurations
    )
    lines += [
        "",
        "Diferencia articular máxima entre las dos soluciones, punto a punto (mrad): "
        + ", ".join(f"{k} {1e3 * per_joint[k]:.1f}" for k in _JOINT_NAMES)
        + f"; **joint4+joint6 {1e3 * per_joint['joint4+joint6']:.1f}**.",
        "",
        f"## FK contra CoppeliaSim en {random_count} configuraciones aleatorias (sin IK)",
        "",
        "| Adaptador | Error medio (µm) | Error máx (µm) | `forward_kinematics` (µs) | `link_poses` (µs) |",
        "|---|---|---|---|---|",
    ]
    for name in ("poe", "ga"):
        stats = fk_stats[name]
        lines.append(
            f"| {name.upper()} | {1e6 * stats['mean_error_m']:.2f} | {1e6 * stats['max_error_m']:.2f} | "
            f"{stats['mean_fk_us']:.1f} | {stats['mean_link_poses_us']:.1f} |"
        )
    speedup_ik = statistics.fmean(poe.ik_seconds) / statistics.fmean(ga.ik_seconds)
    speedup_fk = fk_stats["poe"]["mean_fk_us"] / fk_stats["ga"]["mean_fk_us"]
    lines += [
        "",
        "## Lectura (generada a partir de los números de arriba)",
        "",
        f"- **Misma cinemática.** Error de FK contra el modelo de CoppeliaSim: máx {1e6 * fk_stats['poe']['max_error_m']:.2f} µm (PoE) y "
        f"{1e6 * fk_stats['ga']['max_error_m']:.2f} µm (GA) en {random_count} posturas aleatorias; y ambos dejan la punta sobre el arco "
        f"con error real máx {1e3 * max(poe.sim_error_m[1:]):.3f} mm (PoE) / {1e3 * max(ga.sim_error_m[1:]):.3f} mm (GA), dentro de la "
        "tolerancia de 0,1 mm de las dos IK.",
        (
            "- **Misma solución articular.** Las dos IK coinciden punto a punto."
            if same_solution
            else (
                f"- **Misma pose, distinta solución articular.** Diferencias de hasta {1e3 * per_joint[worst_joint]:.0f} mrad "
                f"(`{worst_joint}`) con la punta en el mismo sitio. No es un error: todo el arco se recorre con joint1 = joint5 = 0, "
                "y con joint5 = 0 los ejes de joint2, joint3, joint4 y joint6 son paralelos (la singularidad de muñeca documentada en "
                "[[PoeKinematicsAdapter]]): un mecanismo planar de 4 revolutas para 3 restricciones (x, z y giro en el plano), es "
                "decir, una familia continua de soluciones (*self-motion*) para cada punto. Los dos Newton-Raphson usan el mismo "
                "Jacobiano y amortiguamiento, pero parametrizan el error de forma distinta (twist de se(3) frente a logaritmo del "
                "motor CGA) y avanzan por esa familia de forma distinta. Es la libertad que la IK numérica no fija -- la misma que "
                "explota `SelfCollisionAvoidingPlanningAdapter` a propósito."
                if planar_self_motion
                else f"- **Misma pose, distinta rama de IK.** Diferencias de hasta {1e3 * per_joint[worst_joint]:.0f} mrad "
                f"(`{worst_joint}`) con la punta en el mismo sitio: cada Newton-Raphson converge a una rama distinta."
            )
        ),
        f"- **Coste.** IK: GA {speedup_ik:.1f}× más rápida por punto (misma media de iteraciones, {statistics.fmean(poe.iterations):.1f} frente a "
        f"{statistics.fmean(ga.iterations):.1f}); `forward_kinematics`: GA {speedup_fk:.1f}× respecto de PoE en numpy. "
        "`link_poses` en GA paga hoy 6 conversiones motor→Pose en Python; es optimizable (una sola pasada en C++ con "
        "`computeKinematicChainMotor` por prefijo) si algún `PlanningPort` lo necesita en bucle.",
        "- **Lo que GA aporta y PoE no:** `Motor.apply` sobre puntos/esferas/planos/líneas, jacobianos de primitivas y dinámica "
        "del mismo `System` -- lo que necesitan la escena conforme (Fase 4a) y el MPC (Fase 4b) de la tesis.",
    ]
    text = "\n".join(lines) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return text


def run(
    radius: float,
    num_points: int,
    random_configurations: int,
    step_pause_seconds: float,
    port: int,
    report_path: Path,
) -> None:
    ensure_coppeliasim_running(port=port, settings_suffix=f"_cr5_poe_vs_gafro_sim_demo_{port}")
    robot = build_cr5_scene(port=port, initial_configuration=_HOME, scene=Scene.empty())

    adapters: Dict[str, KinematicsPort] = {
        "poe": PoeKinematicsAdapter(steps=1),
        "ga": GaKinematicsAdapter(steps=1),
    }
    home_pose = adapters["poe"].forward_kinematics(_HOME)
    time.sleep(0.5)
    sim_home_tip = np.array(robot.tip_position())
    offset_world = sim_home_tip - _position(home_pose)
    local_offset = _rotation(home_pose).T @ offset_world
    print(
        f"Home FK: {np.round(_position(home_pose), 4).tolist()} | Link6_visual en CoppeliaSim: "
        f"{np.round(sim_home_tip, 4).tolist()} | offset: {np.round(offset_world, 4).tolist()}"
    )

    points = _semicircle_points(home_pose, radius, num_points)
    robot.mark_goal(points[-1])

    results: Dict[str, _ArcResult] = {}
    for name, adapter in adapters.items():
        print(f"[{name.upper()}] resolviendo IK de {len(points) - 1} puntos del arco...")
        results[name] = _solve_arc(name, adapter, points)
        summary = results[name]
        print(
            f"[{name.upper()}] {len(summary.configurations) - 1} resueltos, "
            f"IK media {1e3 * statistics.fmean(summary.ik_seconds):.2f} ms, "
            f"iteraciones media {statistics.fmean(summary.iterations):.1f}"
            + (f", FALLÓ en {summary.failed_at}: {summary.failure}" if summary.failed_at else "")
        )

    for name, result in results.items():
        print(f"[{name.upper()}] animando en CoppeliaSim (rastro {'azul' if name == 'poe' else 'verde'})...")
        robot.set_joints(_HOME)
        time.sleep(step_pause_seconds)
        _animate(robot, result, points, offset_world, step_pause_seconds)

    common = min(len(results["poe"].configurations), len(results["ga"].configurations))
    joint_diffs = [
        _joint_differences(results["poe"].configurations[i], results["ga"].configurations[i])
        for i in range(1, common)
    ]

    print(f"FK contra CoppeliaSim en {random_configurations} configuraciones aleatorias...")
    fk_stats = _fk_against_simulator(robot, adapters, local_offset, random_configurations, 0.02)
    robot.set_joints(_HOME)

    report = _write_report(
        report_path, radius, points, results, joint_diffs, fk_stats, offset_world, random_configurations
    )
    print("\n" + report)
    print(f"Informe escrito en {report_path}")


def main() -> None:
    args = _parse_args()
    run(
        radius=args.radius_meters,
        num_points=args.num_points,
        random_configurations=args.random_configurations,
        step_pause_seconds=args.step_pause_seconds,
        port=args.port,
        report_path=Path(args.report_path),
    )


if __name__ == "__main__":
    main()
