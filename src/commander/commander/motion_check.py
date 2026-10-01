"""Revisión de un movimiento ANTES de ejecutarlo, a partir de sus waypoints
planificados en seco (`Manipulator.dry_run`): el primer embrión del
guardián (verify-then-act, ver "Roles del Commander" en el vault).

Mira lo que puede salir mal sin que ningún otro sitio lo detecte:
- un salto grande de una articulación entre dos waypoints seguidos (el
  adaptador real valida límites absolutos, no saltos: Bloque 0, #114);
- la herramienta bajando más de la cuenta (la brida o el punto de agarre
  por debajo de la altura mínima permitida);
- y resume el movimiento (recorrido por articulación, pose final) para
  que una persona decida.

También compara lo planificado con lo medido después (`compare`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from shared_kernel import JointConfiguration, KinematicsPort, Pose

from .manipulation import tool_axis


@dataclass(frozen=True)
class MotionLimits:
    """`max_step_deg`: salto máximo de una articulación entre waypoints
    seguidos. `min_tool_z`: altura mínima (marco de la base) para la brida
    y para el punto de agarre. None: sin comprobar altura."""

    max_step_deg: float = 5.0
    min_tool_z: Optional[float] = None


@dataclass(frozen=True)
class MotionReport:
    waypoints: int
    max_step_deg: float
    max_step_joint: str
    travel_deg: Dict[str, float]
    min_tool_z: float
    final_pose: Pose
    problems: Tuple[str, ...] = field(default_factory=tuple)

    @property
    def ok(self) -> bool:
        return not self.problems

    def summary(self) -> str:
        p = self.final_pose
        travel = ", ".join(f"{j} {v:+.1f}°" for j, v in self.travel_deg.items() if abs(v) >= 0.05) or "nada"
        lines = [
            f"{self.waypoints} waypoints · salto máximo {self.max_step_deg:.2f}° ({self.max_step_joint})",
            f"recorrido: {travel}",
            f"herramienta: altura mínima {self.min_tool_z:+.3f} m · brida final ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})",
        ]
        lines += [f"PROBLEMA: {problem}" for problem in self.problems]
        return "\n".join(lines)


def analyze(
    start: JointConfiguration,
    waypoints: Sequence[JointConfiguration],
    kinematics: KinematicsPort,
    grasp_offset: float,
    limits: MotionLimits,
) -> MotionReport:
    names = [position.joint_name for position in start.positions]
    previous = start
    max_step, max_joint = 0.0, names[0]
    lowest = math.inf
    problems: List[str] = []
    for configuration in [start, *waypoints]:
        pose = kinematics.forward_kinematics(configuration)
        axis = tool_axis(pose)
        tip_z = pose.z + grasp_offset * axis[2]
        lowest = min(lowest, pose.z, tip_z)
        for name in names:
            step = abs(math.degrees(configuration.angle_of(name) - previous.angle_of(name)))
            if step > max_step:
                max_step, max_joint = step, name
        previous = configuration
    final = waypoints[-1] if waypoints else start
    travel = {n: math.degrees(final.angle_of(n) - start.angle_of(n)) for n in names}
    if max_step > limits.max_step_deg:
        problems.append(f"salto de {max_step:.1f}° en {max_joint} entre dos waypoints (máximo {limits.max_step_deg}°)")
    if limits.min_tool_z is not None and lowest < limits.min_tool_z:
        problems.append(f"la herramienta baja a z = {lowest:+.3f} m (mínimo permitido {limits.min_tool_z:+.3f} m)")
    if not waypoints:
        problems.append("el plan no tiene ningún waypoint")
    return MotionReport(len(waypoints), max_step, max_joint, travel, lowest,
                        kinematics.forward_kinematics(final), tuple(problems))


def compare(planned: JointConfiguration, measured: JointConfiguration, kinematics: KinematicsPort) -> Tuple[float, float]:
    """(error articular máximo en grados, error de posición de la brida en
    mm) entre lo planificado y lo medido."""
    joint_error = max(
        abs(math.degrees(planned.angle_of(p.joint_name) - measured.angle_of(p.joint_name)))
        for p in planned.positions
    )
    a, b = kinematics.forward_kinematics(planned), kinematics.forward_kinematics(measured)
    return joint_error, 1000 * math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
