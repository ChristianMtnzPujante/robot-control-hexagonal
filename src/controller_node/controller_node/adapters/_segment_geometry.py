"""Geometría de segmento-vs-esfera compartida entre los planificadores
mínimos de evitación de esta rama (`obstacle_avoiding_planning_adapter.py`,
que solo mira el segmento del tip, y
`whole_body_obstacle_avoiding_planning_adapter.py`, que mira uno por cada
eslabón del robot) -- extraído para no duplicar la misma matemática dos
veces. Puramente numérico (`numpy`), sin depender de `shared_kernel` salvo
por el tipo `SphereObstacle`.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from shared_kernel import SphereObstacle


def closest_point_on_segment(
    start: np.ndarray, end: np.ndarray, point: np.ndarray
) -> np.ndarray:
    segment = end - start
    segment_length_sq = float(segment @ segment)
    if segment_length_sq < 1e-12:
        return start
    t = float(np.clip((point - start) @ segment / segment_length_sq, 0.0, 1.0))
    return start + t * segment


def lateral_direction(segment: np.ndarray) -> np.ndarray:
    """Dirección perpendicular al segmento para rodear el obstáculo cuando
    el centro cae exactamente sobre la recta (caso degenerado: el vector
    "lejos del centro" tiene norma ~0). `segmento × arriba_del_mundo` da un
    lateral horizontal; si el segmento ya es vertical, cae a +X."""
    world_up = np.array([0.0, 0.0, 1.0])
    lateral = np.cross(segment, world_up)
    norm = np.linalg.norm(lateral)
    if norm < 1e-6:
        return np.array([1.0, 0.0, 0.0])
    return lateral / norm


def worst_intersection(
    start: np.ndarray,
    end: np.ndarray,
    obstacles: List[SphereObstacle],
    clearance: float,
) -> Optional[Tuple[SphereObstacle, np.ndarray]]:
    """De entre los obstáculos que invaden el segmento start->end (a menos
    de radius+clearance), el que más lo invade."""
    worst: Optional[Tuple[SphereObstacle, np.ndarray, float]] = None
    for obstacle in obstacles:
        center = np.array([obstacle.center.x, obstacle.center.y, obstacle.center.z])
        closest = closest_point_on_segment(start, end, center)
        distance = float(np.linalg.norm(closest - center))
        required = obstacle.radius + clearance
        penetration = required - distance
        if penetration <= 0:
            continue
        if worst is None or penetration > worst[2]:
            worst = (obstacle, center, penetration)
    if worst is None:
        return None
    return worst[0], worst[1]


def segment_clears_obstacles(
    start: np.ndarray,
    end: np.ndarray,
    obstacles: List[SphereObstacle],
    clearance: float,
) -> bool:
    return worst_intersection(start, end, obstacles, clearance) is None


def detour_point(
    start: np.ndarray,
    end: np.ndarray,
    obstacle: SphereObstacle,
    center: np.ndarray,
    clearance: float,
) -> np.ndarray:
    closest = closest_point_on_segment(start, end, center)
    away = closest - center
    distance = float(np.linalg.norm(away))
    required = obstacle.radius + clearance
    if distance > 1e-6:
        direction = away / distance
    else:
        direction = lateral_direction(end - start)
    return center + direction * required
