---
tags: [arquitectura, adaptador]
---

# _segment_geometry (geometría compartida)

Módulo de geometría pura (`numpy`), compartido entre
[[ObstacleAvoidingPlanningAdapter]] (mira solo el segmento del TIP),
[[WholeBodyObstacleAvoidingPlanningAdapter]] (mira un segmento por cada
eslabón del robot contra obstáculos EXTERNOS) y, desde el 08/09,
[[SelfCollisionAwarePlanningAdapter]] (mira cada eslabón contra los DEMÁS
eslabones del propio robot) — extraído para no duplicar la misma
matemática varias veces. Código:
`src/controller_node/controller_node/adapters/_segment_geometry.py`.

Agnóstico del marco de referencia: opera sobre `np.ndarray` (x,y,z)
puros, sin depender de `shared_kernel` salvo por el tipo
`SphereObstacle`. En la práctica, ambos consumidores le pasan siempre
coordenadas cartesianas relativas a `base_link` — el mismo marco que usa
`PoeKinematicsAdapter` para `forward_kinematics`/`link_poses`/`goal` —
pero el módulo en sí no lo garantiza ni lo impone; si un `SphereObstacle`
se define en coordenadas de mundo de CoppeliaSim "a ojo", solo coincide
con lo que ve el planificador si `base_link` está en el origen del mundo
(cierto hoy, mismo hallazgo de `coppeliasim_scene_builder.py`, pero no
algo que este módulo verifique).

## Funciones

- **`closest_point_on_segment(start, end, point)`** — punto del segmento
  más cercano a `point`: proyección escalar de `(point - start)` sobre el
  segmento, recortada a `[0,1]` para no salirse de él. Caso degenerado
  (`start == end`): devuelve `start` directamente.
- **`lateral_direction(segment)`** — vector unitario perpendicular
  horizontal al segmento (`segmento × arriba_del_mundo`), usado solo como
  *fallback* cuando el centro de un obstáculo cae justo sobre la recta y
  el vector "lejos del centro" no tiene norma fiable. Si el segmento ya
  es vertical, cae a `+X` por defecto.
- **`worst_intersection(start, end, obstacles, clearance)`** — de entre
  los obstáculos que invaden el segmento (distancia centro↔segmento
  menor que `radius + clearance`), devuelve el que más invade
  (`penetration` máxima), o `None` si ninguno invade.
- **`segment_clears_obstacles(start, end, obstacles, clearance)`** —
  azúcar sobre `worst_intersection`: `True` si no hay ninguna invasión.
- **`detour_point(start, end, obstacle, center, clearance)`** — punto de
  rodeo exactamente a `radius + clearance` del centro del obstáculo, en
  la dirección "lejos del centro" (o `lateral_direction` en el caso
  degenerado en que esa dirección no es fiable).
- **`closest_points_between_segments(p1, q1, p2, q2)`** (08/09) — el par
  de puntos más cercanos entre dos segmentos (uno de cada), algoritmo
  cerrado clásico (Lumelsky 1985 / Ericson, *Real-Time Collision
  Detection* §5.1.9). A diferencia de `closest_point_on_segment`
  (segmento vs. un ÚNICO punto), aquí AMBOS lados son segmentos — lo que
  hace falta para autocolisión, donde los dos "obstáculos" son eslabones
  del propio robot, no esferas.
- **`segment_segment_distance(p1, q1, p2, q2)`** (08/09) — azúcar sobre
  `closest_points_between_segments`: solo la distancia. Dos segmentos que
  comparten un extremo dan `0.0` — no es automáticamente "colisión", ver
  [[SelfCollisionAwarePlanningAdapter]] sobre cómo se excluyen esos pares
  antes de interpretar el resultado.

## `radius + clearance`

Aparece en varias funciones: `radius` es el tamaño real del
`SphereObstacle` (metros); `clearance` es un margen de seguridad
ADICIONAL, elegido al construir el planificador, independiente del
tamaño del obstáculo. La distancia mínima exigida entre el punto más
cercano de un segmento y el CENTRO del obstáculo es la suma de ambos, no
solo el radio — no basta con no tocar la superficie del obstáculo, hay
que quedarse `clearance` metros más allá de ella.

## Cómo lo usa cada consumidor

- [[ObstacleAvoidingPlanningAdapter]] llama a estas funciones sobre UN
  único segmento: el del TIP.
- [[WholeBodyObstacleAvoidingPlanningAdapter]] las llama sobre **todos**
  los segmentos del cuerpo (`base_link` → primera articulación → ... →
  tip, uno por eslabón real, construidos por su propio helper interno
  `_body_segments` a partir de `KinematicsPort.link_poses`) y en **cada**
  waypoint de la trayectoria, no solo en el final — misma matemática,
  muchas más llamadas por intento de desvío.

## Ver también

- [[ObstacleAvoidingPlanningAdapter]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[SelfCollisionAwarePlanningAdapter]]
- [[PlanningPort]]
