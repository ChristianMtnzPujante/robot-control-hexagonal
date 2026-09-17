---
tags: [arquitectura, adaptador]
---

# SelfCollisionAwarePlanningAdapter

Tercer implementador de [[PlanningPort]] — a diferencia de
[[ObstacleAvoidingPlanningAdapter]]/[[WholeBodyObstacleAvoidingPlanningAdapter]]
(evitación de obstáculos EXTERNOS), este comprueba AUTOcolisión: que
ningún eslabón del robot choque con otro eslabón del propio robot, haya o
no obstáculos externos de por medio. Código:
`src/controller_node/controller_node/adapters/self_collision_planning_adapter.py`.

Motivado por un hallazgo real (08/09): `GetErrorID()` = `[76]` del CR5
físico durante [[Scripts de Demostración|cr5_semicircle_demo.py]] — "el
extremo interfiere con el cuerpo del robot" (manual oficial + tabla de
alarmas del fabricante, nivel de severidad 5). El propio driver del CR5
ya detiene el robot cuando pasa, pero para entonces ya se había mandado un
`MovJ` real. Este adaptador comprueba ANTES de mandar nada — rechaza la
trayectoria ENTERA con un error explícito si algún waypoint colisionaría,
mismo criterio que [[PoeKinematicsAdapter]] (IK que no converge) y
`_within_a_full_turn` en [[WholeBodyObstacleAvoidingPlanningAdapter]]
(ángulo fuera de ±2π): fallar alto y explícito, no mandar algo peligroso.
Deliberadamente NO intenta rodear la autocolisión — eso lo hace su
hermana, `SelfCollisionAvoidingPlanningAdapter` (mismo archivo, ver más
abajo), añadida el mismo 08/09 a petición del usuario.

## Modelo geométrico: cápsulas

Cada eslabón = el segmento entre dos poses consecutivas de
`KinematicsPort.link_poses` (mismo `_body_segments` que
[[WholeBodyObstacleAvoidingPlanningAdapter]]) + un radio fijo. Dos
eslabones colisionan si la distancia entre sus segmentos
(`_segment_geometry.segment_segment_distance`, ver
[[_segment_geometry (geometría compartida)]]) es menor que la suma de sus
radios — aquí, el doble del mismo radio único.

### Radio calibrado contra dos puntos reales, no adivinado

El URDF del CR5 solo trae mallas como geometría de colisión (`Link1.dae`
...`Link6.dae`), sin primitivas con radio — no hay un número que leer sin
procesar las mallas a mano. En vez de un valor a ciegas, se midió contra
dos referencias reales:

- **Home** (segura por definición): la distancia real más corta entre
  eslabones no adyacentes (`joint3→joint4` vs `joint5→joint6`) es
  **11.6cm**.
- **La secuencia de 10 waypoints que disparó la alarma real**: esa misma
  distancia bajó progresivamente hasta **7.7cm** en el último tramo
  calculado (joint4≈-143°, joint6≈89°) — la zona exacta del incidente.

`_DEFAULT_LINK_RADIUS_METERS = 0.04` (2×0.04 = 8cm de separación exigida)
cae en el hueco entre ambas: dejar la home segura con 3.6cm de margen, y
detectar la trayectoria real problemática ANTES del último tramo (punto
8/9, no solo en el punto exacto del incidente) — verificado reproduciendo
la secuencia real completa contra `PoeKinematicsAdapter`.

## Exclusión de pares — geométrica, no por índice

Un hallazgo intermedio real: excluir solo "el índice siguiente" (i, i+1)
NO basta. El CR5 tiene `joint1`/`joint2` en el MISMO punto físico (offset
cero entre ellos en el URDF — dos ejes que se cruzan en el "hombro"), así
que el eslabón `base→joint1` y el eslabón `joint2→joint3` comparten un
extremo aunque sus índices no sean consecutivos — con una exclusión
ingenua por índice, ese par salía "en colisión" en TODAS las
configuraciones, incluida la propia home (falso positivo permanente,
encontrado antes de llegar al radio calibrado de arriba).

`structural_adjacency_exclusions(reference_link_poses)` es la exclusión
real: dos segmentos se excluyen si comparten un extremo (a menos de 1mm)
en una configuración de referencia CUALQUIERA — es una propiedad de la
geometría del robot (`RobotDescription`), no de la postura, así que basta
calcularlo una vez (la primera llamada a `compute_trajectory`, con
cualquier configuración) y cachearlo. Cubre tanto los pares consecutivos
normales como el caso `joint1`/`joint2` de arriba, con el mismo mecanismo.

## `scene` se ignora a propósito

Igual que `NaivePlanningAdapter` ignora la `Scene` para su único
problema (cablear el puerto), este adaptador resuelve solo autocolisión —
combinarlo con evitación de obstáculos externos es responsabilidad de
quien componga varios `PlanningPort`, no de este adaptador.

## `SelfCollisionAvoidingPlanningAdapter` — la misma comprobación, pero intenta esquivar (08/09)

A petición del usuario ("necesitamos generar una trayectoria en la que no
colisione, por los mismos puntos"): en vez de solo rechazar, reintenta la
IK del MISMO objetivo cartesiano desde una semilla distinta antes de
rendirse. No es una búsqueda general (no hay RRT/CHOMP todavía, Bloque 4)
— es un truco concreto de brazos con **muñeca esférica de 3 ejes** como el
CR5:

- Cerca de `joint5≈0` (la singularidad YA documentada en
  [[PoeKinematicsAdapter]] — ejes de `joint4`/`joint6` casi paralelos ahí)
  hay un grado de libertad casi redundante entre esos dos joints para una
  orientación dada: moverlos en direcciones OPUESTAS apenas cambia la
  orientación final del tip, pero SÍ desplaza la posición del tramo
  intermedio (`joint4→joint5` no tiene longitud cero) — justo la palanca
  que hace falta para separar la muñeca del antebrazo sin mover el tip.
- `_wrist_joint_pair(configuration)` — `(positions[-3], positions[-1])`,
  saltando el joint del medio (la propia singularidad). Genérico por
  posición en la cadena, no hardcodeado a los nombres del CR5 — con menos
  de 3 joints devuelve `None` y el adaptador se comporta exactamente como
  `SelfCollisionAwarePlanningAdapter` (nada que desplazar).
- Reintenta con desplazamientos crecientes (`nudge_step_degrees`, por
  defecto 2°, hasta `max_nudge_degrees`, por defecto 90°), probando ambos
  signos en cada magnitud, hasta encontrar una semilla que converja Y dé
  una configuración libre de autocolisión — o agota el rango y lanza
  `SelfCollisionError` igual que la versión "aware".

**Verificado contra el incidente real completo** (08/09): reproduciendo
los 10 puntos exactos de `cr5_semicircle_demo.py` que dispararon la
alarma física, un desplazamiento de solo **4°** ya basta para el punto que
colisionaba — con un salto adicional de apenas ~10° respecto al waypoint
anterior (nada que ver con una "vuelta de muñeca" completa, ~180°, que sí
resuelve el mismo punto pero con saltos de 85-290° entre waypoints,
inutilizable para un movimiento suave). Las 9 configuraciones del arco
completo se alcanzan sin excepción, con el mismo punto cartesiano final
exacto (error de posición <0.001mm) — test
`test_avoiding_adapter_finds_a_self_collision_free_branch_for_the_real_incident`.
Ya wireado en [[Scripts de Demostración|cr5_semicircle_sim_demo.py y
cr5_semicircle_demo.py]] en lugar de `PoeKinematicsAdapter` a secas.

Limitación conocida, no resuelta: reconstruye la trayectoria como
`[current_configuration, solución]` — si el `KinematicsPort` envuelto
interpola en varios pasos (`steps>1`), esa interpolación se descarta. Pensado
para el uso real de hoy (`PoeKinematicsAdapter(steps=1)`, un punto
cartesiano por llamada), no para trayectorias multi-paso de una sola llamada.

### Efecto secundario del "nudge": el cierre no vuelve al cero exacto (08/09)

Extendido a un CÍRCULO COMPLETO (`cr5_circle_sim_demo.py`/`cr5_circle_demo.py`,
360° en vez de 180°), la primera prueba real reveló un efecto colateral:
la punta cerraba el círculo con <0.05mm de error (perfecto), pero
`joint4`/`joint6` individuales quedaban a 1-2° del valor de partida — el
usuario lo notó a simple vista, el brazo no volvía "recto" del todo.
**No es un problema de calibración del robot** — es la MISMA redundancia
de muñeca que este adaptador explota para esquivar la autocolisión:
varias combinaciones de `joint4`/`joint6` dan la pose IDÉNTICA del tip,
así que tras varios "nudges" a lo largo de la vuelta, la cadena queda
asentada en una combinación distinta de la de partida, aunque
cartesianamente sea la misma pose.

Corregido, no en el adaptador sino en los dos scripts de demo, con
`_snap_to_exact_start_if_needed`: si la última configuración no coincide
con la de partida dentro de 0.5°, añade un waypoint final a la
configuración de partida EXACTA — ya demostrada alcanzable (es de donde
salió todo el recorrido), así que es un movimiento pequeño y seguro, no
un salto nuevo. Verificado en vivo (CoppeliaSim) y contra el arnés de
servidor TCP de mentira (robot real): detecta el desvío de 1.82° y el
último `MovJ`/waypoint pasa a ser literalmente `{0,0,0,0,0,0}`.

## Ver también

- [[PlanningPort]]
- [[_segment_geometry (geometría compartida)]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[Cr5RealRobotAdapter]]
- [[Decisiones de Diseño Clave]]
- [[Scripts de Demostración]]
