---
tags: [arquitectura, adaptador]
---

# PoeKinematicsAdapter

Implementa [[KinematicsPort]] vía Product of Exponentials (Lynch & Park,
*Modern Robotics*) — la única cinemática REAL (no doble de test, no stub)
que resuelve IK de verdad hoy. Código:
`src/controller_node/controller_node/adapters/poe_adapter.py`. Derivación
completa función a función, con diagrama del mecanismo:
`poe_adapter.html` en el mismo directorio.

## De dónde salen los twists

`RobotDescription` (ver [[Scene y Percepción]]) en configuración home →
twist S_i=(w_i, v_i) por articulación: w_i = R_i·axis_i, v_i = -w_i×q_i
(o v_i = R_i·axis_i para prismáticas). Por defecto usa
`_DEFAULT_CR5_DESCRIPTION` — los mismos datos que antes vivían
hardcodeados como `_JOINT_NAMES`/`_JOINT_ORIGINS`, copiados del URDF real
del CR5.

`_validate_cr5_reference_if_applicable` es una red de seguridad TEMPORAL:
si un `RobotDescription` recibido dice ser un CR5 (mismos nombres de
joint), compara sus twists/pose home contra la referencia hardcodeada ya
validada a mano — confiar en la ruta genérica (`urdf_kit.parse_urdf_file`,
Bloque 9) para el único robot real que hay hoy, antes de fiarse de ella
para uno nuevo.

## IK: Newton-Raphson amortiguado, no pseudoinversa pura

`_inverse_kinematics` itera sobre el Jacobiano espacial (vía la Adjunta,
`IKinSpace` de *Modern Robotics* cap. 6), con paso amortiguado
(Levenberg-Marquardt) en vez de pseudoinversa sin más — el CR5 tiene
muñeca esférica y una singularidad real en joint5≈0 (ejes de joint4 y
joint6 paralelos ahí) donde la pseudoinversa sin amortiguar dispara el
paso. Si no converge en `max_iterations`, lanza `RuntimeError` explícito
en vez de devolver una trayectoria hacia un sitio equivocado.

## Hallazgo (08/09): la home no puede subir, solo bajar

Al diseñar un arco cartesiano de prueba (`cr5_semicircle_sim_demo.py`)
manteniendo la orientación fija de la home, se encontró por barrido
empírico (grid de IK real alrededor de la home, con esa misma orientación
fija) que la home está en el límite superior/lateral de lo alcanzable con
su propia orientación: **ningún punto con Z mayor que el de la home
converge, y tampoco ningún punto a la MISMA altura salvo la propia home**
— solo hay margen moviéndose hacia -Z. No es la misma singularidad de
joint5≈0 de más arriba (esa es sobre el Jacobiano en un punto; esto es
sobre la FORMA del subespacio alcanzable con una orientación fija) — puede
ser la misma causa física (home totalmente extendida verticalmente) o
una distinta, sin investigar a fondo todavía. Implicación práctica: cualquier
trayectoria cartesiana que parta de la home con orientación fija debe
alejarse hacia abajo, nunca hacia arriba ni lateralmente a la misma altura,
o `compute_trajectory` lanzará `RuntimeError` desde el primer paso.

### Ampliación (14/09): el porqué, con un número

Al diseñar el gesto de saludo (`cr5_wave_sim_demo.py`, ver
[[Scripts de Demostración]]) se midió lo que en 08/09 solo se había
observado por barrido: **en la home el tip está a 0.933m del hombro**
(hombro = origen de `joint2`, a z=0.147), y el **alcance de catálogo del
CR5 es 0.9m**. La home no está "cerca" del borde del espacio alcanzable —
está en el borde, con el brazo completamente extendido en vertical.

Eso explica de una vez las dos mitades del hallazgo del 08/09 sin
necesidad de invocar ninguna singularidad: no puede subir porque ya está
arriba del todo, y no puede moverse de lado a la misma altura porque
cualquier punto con el mismo `z` y distinto `x` está **todavía más lejos**
del hombro que la propia home (√(x²+y²+(z−0.147)²) crece con |x| a `y`,`z`
fijos). No hay contradicción con la singularidad de `joint5≈0` de más
arriba: son cosas distintas (aquélla es sobre el Jacobiano en un punto,
ésta sobre la forma del subespacio alcanzable), y ésta es puramente de
alcance.

Consecuencia práctica al diseñar trayectorias cartesianas: si el gesto
necesita margen LATERAL (no solo hacia abajo), hay que partir de una
postura retraída, no de la home. La que usa el saludo — `(0, -45, 90, 45,
25, -90)` — deja el tip a **0.647m del hombro, 25cm de margen**.

### Hallazgo (14/09): anclar la rama de la IK en vez de confiar en ella

Segundo hallazgo del mismo gesto, y el más reutilizable de los dos. El
saludo necesita la herramienta **inclinada hacia arriba**, que es una
rotación sobre el eje X del mundo. En la postura de saludo ningún joint
del CR5 la da suelto: el eje de `joint4`/`joint6` es el eje Y del mundo y
el de `joint5` es el eje Z.

El camino obvio era pedirlo por IK: rotar la pose "plana" sobre X y dejar
que Newton-Raphson encontrara la postura. **Converge, y aun así no
sirve**: la rama a la que llega es contorsionada (`joint1`=-9°,
`joint6`=-69°) y, al barrer el saludo desde ahí, la IK va saltando de rama
punto a punto — **saltos de hasta 51° entre waypoints consecutivos**, con
un `cierre` final de 0.00° que no delata nada del problema.

La solución no fue tocar la IK sino no usarla para eso. Conjugar una
rotación sobre Z por ±90° sobre Y la convierte en una rotación sobre X:

> **Ry(−90)·Rz(β)·Ry(90) = Rx(−β)**

Así que con `joint4`=+45 y `joint6`=−90 fijos, **`joint5` pasa a SER
directamente el ángulo de cabeceo** — exacto, analítico, sin IK
(verificado numéricamente: ~1e-6 de error de orientación frente al
objetivo). Con la postura anclada así, la IK solo tiene que seguir deltas
pequeños a lo largo del barrido y el paso máximo baja de 51° a ~7°.
`joint1`/`joint5`/`joint6` quedan además congelados durante todo el gesto.

Es la misma lección que ya había dado `_snap_to_exact_start_if_needed` en
el círculo (ver [[Scripts de Demostración]]), desde el otro lado: **donde
hay redundancia, conviene fijar la rama explícitamente en vez de confiar
en que la IK elija siempre la misma**. Allí se arreglaba a posteriori (un
waypoint final a la configuración exacta); aquí se evita de entrada (la
postura de partida se define en espacio de articulaciones, no por su
pose).

## Cinemática directa: parte formal de KinematicsPort desde el 08/09

- `forward_kinematics(configuration) -> Pose` — cinemática directa del
  tip, mismo marco (`base_link`) que exige `goal`. La usa
  [[ObstacleAvoidingPlanningAdapter]] para saber dónde está el robot
  AHORA en cartesiano.
- `link_poses(configuration) -> List[Pose]` — pose de CADA articulación,
  no solo el tip (`link_poses(...)[-1] == forward_kinematics(...)` por
  construcción, exacto en PoE porque `RobotDescription` no tiene ningún
  eslabón estático tras la última articulación). La usa
  [[WholeBodyObstacleAvoidingPlanningAdapter]] para comprobar colisiones
  de cuerpo completo, no solo del tip.

Hasta el 08/09 eran capacidades extra fuera de [[KinematicsPort]], y los
dos planificadores que las necesitan las exigían vía su propio `Protocol`
local (`_KinematicsPortWithForward`/`_KinematicsPortWithLinkPoses`). Ahora
viven en el puerto formal — [[CoppeliaSimIkKinematicsAdapter]] las
implementa también desde entonces; ver detalle por adaptador en
[[KinematicsPort]] y el motivo en [[Decisiones de Diseño Clave]].

## Ver también

- [[KinematicsPort]]
- [[ObstacleAvoidingPlanningAdapter]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[Scene y Percepción]] — `RobotDescription`
- [[CR5 vs Panda (Generalización)]] — prueba real con otro robot, 7 GDL
