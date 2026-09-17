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

## Lo que deja preparado (tesis)

`self._system` es un `pygafro.System` completo: jacobianos de primitivas
(`getEEPrimitiveJacobian`), `Motor.apply` sobre puntos/esferas/planos/
líneas, matriz de masas y dinámica. Es lo que necesitan la escena
conforme (Fase 4a, ver [[Primitivas Geométricas]] sobre bounded contexts
separados) y el MPC conforme (Fase 4b).

Pendiente, igual que en PoE: límites articulares, elección de rama, y la
IK geométrica cerrada de `docs/algebra_geometrica_conforme.md` §5.
