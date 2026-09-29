---
tags: [arquitectura, adaptador]
---

# GaKinematicsAdapter

Implementa [[KinematicsPort]] con **álgebra geométrica conforme (CGA)**
vía `gafro`/`pygafro` (Löw, Abbet & Calinon, Idiap, IEEE T-RO 2023). Real
desde el **17/09/2026** (F1.2 de la tesis): antes era un stub con
`NotImplementedError` esperando a "compilar pygafro". Código:
`src/controller_node/controller_node/adapters/ga_adapter.py`; tests:
`src/controller_node/test/test_ga_adapter.py` (8 tests, todos cruzados
contra [[PoeKinematicsAdapter]]).

## Por qué ya no hay nada que compilar

La prueba de viabilidad F1.1 (`~/Desktop/doctorado/informe_F1_1_viabilidad_pygafro.md`)
demostró que `pygafro` se instala como rueda de PyPI
(`python3 -m pip install --user pygafro`, versión 1.3.5) sobre el Python
3.10 del sistema, y que la FK del CR5 así obtenida coincide con PoE a
precisión máquina. Compilarlo desde fuente también funciona (26 min) pero
no hace falta. El módulo importa `pygafro` de forma **perezosa** (dentro
de `__init__`), así que `controller_node` sigue arrancando con las demás
estrategias aunque la rueda no esté instalada; solo `strategy="ga"` falla,
con un mensaje que dice qué instalar.

## Cómo se construye el robot

Mismos datos crudos que PoE: `RobotDescription` (origen xyz/rpy + eje por
articulación, ver [[urdf_kit y RobotDescription]]). Cada
`JointDescription` se convierte en una articulación de un `pygafro.System`:

- **frame** = motor fijo F_i = T·R del `<origin>` URDF
  (`Motor(Translator, Rotor.fromQuaternion)`),
- **eje** = `RotorGenerator([z, -y, x])` para revolutas (bivector del eje)
  o `TranslatorGenerator([x, y, z])` para prismáticas.

Las articulaciones se encadenan (`setParentLink`/`setChildLink`) y se
crea una `KinematicChain` explícita con todas ellas -- sin usar las
clases `Manipulator_N` de pygafro, que están limitadas a N ≤ 10. Sin
robot dado, usa `_DEFAULT_CR5_DESCRIPTION` de `poe_adapter.py` (única
fuente de la tabla del CR5, compartida a propósito).

Las convenciones de pygafro (orden de componentes del bivector, orden del
cuaternión, T·R) **no están documentadas upstream**; se fijaron
empíricamente contra Rodrigues en F1.1 y quedan anotadas en el docstring
del módulo y en [[Decisiones de Diseño Clave]].

## Qué hace cada método

- `forward_kinematics`: M(θ) = Π F_i·R_i(θ_i) de la cadena
  (`computeKinematicChainMotor`) → `Pose` (traslación de la matriz del
  motor + cuaternión del rotor).
- `link_poses`: producto acumulado de `joint.getMotor(θ_i)`; coincide con
  PoE en las 6 articulaciones (error < 1e-12).
- `compute_trajectory`: IK por Newton-Raphson amortiguado
  (Levenberg-Marquardt), **mismo esquema, mismos parámetros y mismas
  tolerancias que PoE** para que `strategy="poe"` y `strategy="ga"` sean
  intercambiables y comparables. La diferencia es el álgebra: el error es
  el logaritmo del motor E = M_goal·M̃(θ) (un bivector de 6 componentes) y
  el Jacobiano es el geométrico de gafro (columna i = generador de la
  articulación i, misma base), así que el paso J^T(JJ^T+λ²I)^{-1}·log(E)
  no sale del álgebra. La parada se decide sobre el error geométrico real
  (distancia del tip y ángulo del rotor de error), porque el logaritmo
  del motor en gafro no es el twist de se(3) en su parte traslacional.
- `last_iteration_count`: diagnóstico (no parte del puerto), lo usa la
  comparativa. PoE tiene el mismo atributo desde el mismo día.
- `last_trace` (29/09): una `IkIteration` por iteración de Newton-Raphson
  (θ, error, distancia de la punta, ángulo, paso), también solo
  diagnóstico. Definida en `poe_adapter.py` y compartida con PoE; el error
  se guarda en el orden del twist (wx, wy, wz, vx, vy, vz) con
  `generator_to_twist_order`, para restar las dos trazas directamente.

## Comparativa PoE vs GA en CoppeliaSim (17/09)

`commander/cr5_poe_vs_gafro_sim_demo.py` (ver [[Scripts de Demostración]])
resuelve el mismo arco de `cr5_semicircle_sim_demo.py` con los dos
adaptadores y contrasta ambos con la posición de `Link6_visual` que
calcula el propio CoppeliaSim. Informe generado:
`docs/comparativa_poe_vs_gafro_coppeliasim.md`. Resumen:

| | PoE | GA |
|---|---|---|
| Error real de la punta en CoppeliaSim (máx, arco) | 0,080 mm | 0,089 mm |
| Error de FK vs CoppeliaSim (200 posturas aleatorias) | 0,00 µm | 0,00 µm |
| Iteraciones de IK por punto (media) | 3,1 | 3,1 |
| Tiempo de IK por punto (media) | 0,77 ms | 0,28 ms |
| `forward_kinematics` | 188 µs | 41 µs |
| `link_poses` | 118 µs | 248 µs |

Hallazgo que no es un error: las dos IK dan **soluciones articulares
distintas** (hasta 274 mrad en `joint6`) para la **misma pose**. El arco se
recorre con `joint1 = joint5 = 0`, y con `joint5 = 0` los ejes de
joint2/3/4/6 son paralelos: un mecanismo planar de 4R para 3 restricciones,
es decir, una familia continua de soluciones por punto (*self-motion*).
Cada Newton-Raphson avanza por esa familia según cómo parametriza el
error (twist de se(3) frente a log del motor). Es la misma libertad que
ya explota `SelfCollisionAvoidingPlanningAdapter` y la misma moraleja del
14/09: donde hay redundancia, la rama se elige por diseño, no se delega en
el solucionador.

`link_poses` en GA es más lento porque hace 6 conversiones motor→`Pose`
en Python; optimizable si un `PlanningPort` lo necesita en bucle.

## Dónde se separan exactamente los dos cálculos (29/09)

`cr5_poe_vs_gafro_simple_demo.py` (ver [[Scripts de Demostración]]) compara
los internos de los dos adaptadores paso a paso. Lo que sale:

| Paso | PoE | GA | ¿Igual? |
|---|---|---|---|
| Modelo | twist S_i = (w; v) en la base | eje (bivector local) + motor fijo F_i | mismos datos; FK en la home idéntica (5e-16) |
| FK | e^[S1]θ1···M, matriz 4x4 | Π F_i·R_i(θ_i), motor | sí (< 1e-9 µm) |
| Jacobiano | Ad_T·S_i | M·B_i·M̃, base [e12,e13,e23,e1i,e2i,e3i] | sí, tras reordenar (4e-16) |
| Error, parte rotacional | log de so(3) | log del rotor | sí (1e-10) |
| Error, parte traslacional | v = G(θ)⁻¹·p (tornillo) | t = traslación de M_goal·M̃ tal cual | **solo sin giro** |

Números, desde `[0, 30, -60, 0, 40, 0]°`:

- **Traslación pura** (8 cm en -X, 5 cm en -Z): error inicial idéntico (6e-17), las
  mismas iteraciones (3) y la misma solución (0,15 mrad).
- **Con giro** (objetivo = pose de `[20, 45, -80, 10, 60, 30]°`, 46,5° de
  giro): la traslación del error difiere 0,17. PoE pide v = (0,206, 0,508,
  0,004); GA pide t = (0,070, 0,500, 0,173). GA aleja la punta en la 1ª
  iteración (100 → 197 mm) mientras corrige casi todo el giro, y tarda 6
  iteraciones frente a 4. Acaban en la misma solución (0,11 mrad).

Es decir: gafro calcula `log(T·R)` como si fuera `log T + log R`, que
coincide con el twist solo a primer orden. Converge igual porque el paso
va en la dirección correcta, pero con giros grandes da peores pasos.
Si hiciera falta, bastaría con convertir t → G(θ)⁻¹·t antes del paso para
tener el mismo error que PoE. No se ha hecho, para seguir comparando las
dos álgebras tal como son.

**Matiz al hallazgo del 17/09.** En la home (brazo estirado en vertical y
`joint5 = 0`: Jacobiano de rango 3 de 6), las dos IK acaban 2° separadas,
pero el álgebra no es la causa. Con traslación pura el error es idéntico
y las cuentas coinciden hasta la iteración 2 (separación de 0,001 mrad).
Al salir de la singularidad hay un paso enorme (la punta se va a 412 mm),
el redondeo se amplifica y cada una cae en un punto distinto de la
familia de soluciones. Donde hay redundancia, cualquier diferencia
mínima (del álgebra o del redondeo) elige otra rama. La moraleja del 14/09
se mantiene: la rama se fija por diseño.

## Lo que deja preparado (tesis)

`self._system` es un `pygafro.System` completo: jacobianos de primitivas
(`getEEPrimitiveJacobian`), `Motor.apply` sobre puntos/esferas/planos/
líneas, matriz de masas y dinámica. Es lo que necesitan la escena
conforme (Fase 4a, ver [[Primitivas Geométricas]] sobre bounded contexts
separados) y el MPC conforme (Fase 4b).

Pendiente, igual que en PoE: límites articulares, elección de rama, y la
IK geométrica cerrada de `docs/algebra_geometrica_conforme.md` §5.

**(23/09) Hacia dónde crece.** Hoy el adaptador solo hace lo que pide
[[KinematicsPort]] (FK, `link_poses`, IK de pose a pose). Lo que
distingue a CGA, las tareas por primitivas ("sigue esta línea"), iría en
un puerto aparte que esta misma clase también cumpliría: residuo +
Jacobiano por tarea, construidos con `Motor.apply` sobre `Line`/`Point`
y el Jacobiano geométrico que ya usa `_geometric_jacobian`. Decisión
abierta en [[Decisiones de Diseño Clave]] (23/09). Prueba numérica sobre
el CR5 en `docs/cga_tareas_linea_prioridades.py`, que también fija los
índices de blades de `pygafro.Line`: dirección = `e01i, e02i, e03i`,
momento = `e23i, −e13i, e12i`.
