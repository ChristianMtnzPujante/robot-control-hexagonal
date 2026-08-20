# Álgebra geométrica conforme (CGA): notas prácticas

Extraído de *Geometric Algebra for Computer Science* (Dorst, Fontijne, Mann;
Morgan Kaufmann, 2007) — capítulos 13 y 14, que son la parte del libro
dedicada al modelo conforme (CGA) y sus aplicaciones. Es la misma línea de
trabajo que cita `ga_adapter.py` (Hestenes et al. 1999, y la formalización
posterior en la que se apoya `gafro`/Löw-Abbet-Calinon), así que sirve como
la base matemática de la que `gafro` es la implementación en C++.

Esto responde directamente a la tarea pendiente del Bloque 1 del
`ROADMAP.md`: traducir "plano de la mesa" / "objeto a evitar" a primitivas
CGA concretas.

## 1. El modelo conforme en una página

CGA representa el espacio euclídeo E³ dentro de un espacio de 5 dimensiones
`R^{4,1}` (3 dimensiones euclídeas + 2 extra: el origen `o` y el infinito
`∞`, ambos vectores nulos). La razón de esa sobredimensión: permite que
puntos, planos y esferas sean todos **vectores** del mismo espacio, y que
las transformaciones rígidas (traslación + rotación) sean **versores** —
un único tipo de objeto algebraico que se multiplica, no una matriz 4x4 y un
cuaternión que hay que sincronizar aparte.

La métrica se define para que el producto interno de dos puntos sea
proporcional a menos la distancia euclídea al cuadrado entre ellos:

```
p · q ~ -½ d²(P, Q)
```

De ahí sale todo lo demás. Un punto normalizado a distancia `p` del origen
se representa como:

```
p = o + p + ½p²∞
```

(`o` = origen, `p` = vector euclídeo normal de toda la vida, `∞` = punto
en el infinito, `p²` = `p·p`).

## 2. Las tres primitivas de `geometry_kernel`, en CGA real

`geometry_kernel/primitives.py` hoy usa coordenadas cartesianas planas
(`Point(x,y,z)`, `Plane(point, normal)`, `SphereObstacle(center, radius)`),
documentado explícitamente como provisional. Esto es lo que cambiaría si se
implementa CGA de verdad (vía `gafro`/`pygafro`, cuando esté compilado):

| Primitiva hoy | Forma en CGA (vector de `R^{4,1}`) | Condición que lo caracteriza |
|---|---|---|
| `Point(x,y,z)` | `p = o + p + ½p²∞` | `p² = 0` (vector nulo) |
| `Plane(point, normal)` | dual: `π = n + δ∞` (`n`=normal, `δ`=distancia al origen) | `π² = n²`, `∞·π = 0` |
| `SphereObstacle(center, radius)` | dual: `σ = c - ½ρ²∞` | `σ² = ρ²`, peso `-∞·σ = 1` |

Lo notable: **las tres son el mismo tipo de objeto** (un vector de 5
componentes), solo que con una condición algebraica distinta. Un punto es,
literalmente, una esfera de radio cero. Esto es lo que hace que "unificar
percepción y planificación bajo un mismo `Scene`" tenga sentido matemático
y no sea solo conveniencia de diseño: la Scene de hoy (planos + esferas +
puntos con nombre) ya está calcada de esta estructura, aunque todavía no use
la representación algebraica real.

También existe la representación **directa** (no dual) de un plano, si en
vez de normal+distancia tienes tres puntos sobre él:

```
Π = p ∧ q ∧ r ∧ ∞
```

Útil si `PerceptionPort` en algún momento detecta un plano como "tres
puntos de una nube", en vez de como normal+offset ya ajustados.

## 3. Transformaciones como versores (esto es lo que de verdad vale la pena)

- Traslación: `T_t = exp(-t∞/2) = 1 - t∞/2` (exacto, la serie de Taylor se
  trunca sola).
- Rotación: `R = exp(-Iθ/2)` (`I` = bivector del plano de rotación, `θ` =
  ángulo) — igual que un rotor normal, pero ahora vive en el mismo espacio
  que las traslaciones.
- Movimiento rígido general (traslación + rotación combinadas, lo que en
  robótica se llama un *motor*): `M = T_t · R`.

La propiedad clave, y la razón por la que el libro insiste tanto en ella:
**cualquier cosa construida con los productos del álgebra se transforma
automáticamente bien bajo `M`** — no hay que verificar aparte que una
construcción "siga siendo válida" tras mover el robot. Un plano transformado
por `M` sigue siendo un plano; una esfera transformada por `M` sigue siendo
una esfera con el mismo radio. Esto es lo que en el código actual se hace a
mano (recalcular cada primitiva tras cada movimiento) y en CGA es gratis.

## 4. Cinemática directa: es lo mismo que PoE, en otra álgebra

Esto es el hallazgo más útil de cara al trabajo pendiente en
`poe_adapter.py` y `ga_adapter.py`. El libro construye la cinemática directa
de un brazo (ejemplo: un Puma 560) exactamente con la misma estructura que
el TODO de `poe_adapter.py` ya menciona (twists `S_i = (w_i, v_i)`):

```
T_0 = 1
T_i = T_{i-1} · exp(-t_i∞/2)          # traslación acumulada del eslabón i
A_i = T_{i-1} · B_i · T_{i-1}⁻¹        # bivector de rotación del eslabón i, ya en coordenadas del mundo

# en tiempo de ejecución, con los ángulos θ_i ya conocidos:
M_0 = 1
M_i = M_{i-1} · exp(-A_i θ_i / 2)      # motor acumulado hasta el eslabón i
```

`t_i` (traslación del eslabón) y `B_i` (plano/eje de rotación) son
exactamente `v_i` y `w_i` del *Product of Exponentials* que ya está
documentado como pendiente. **Extraer los twists del CR5 (la tarea real
pendiente de `PoeKinematicsAdapter`) es el mismo trabajo que necesitaría
`GaKinematicsAdapter`** — no son dos tareas de investigación distintas, son
la misma tarea expresada en dos álgebras. Merece la pena implementar PoE
primero (con matrices/vectores normales, más fácil de depurar) y reusar
esos mismos parámetros para el adaptador GA después, en vez de investigar
ambos caminos por separado.

## 5. Cinemática inversa: una alternativa geométrica a Newton-Raphson

El TODO actual de `poe_adapter.py`/`dh_adapter.py` apunta a Newton-Raphson
sobre el Jacobiano (el método clásico, el que se dedujo a mano en teoría).
El libro resuelve IK de otra forma, sin trigonometría hasta el final, con
un ejemplo de un brazo de 2 eslabones (hombro esférico + codo) muy cercano
en estructura a un brazo industrial de 6 ejes con muñeca esférica (que es
casi con toda seguridad la estructura del CR5, como la mayoría de brazos
industriales de 6 ejes):

1. **Esfera hombro ∩ esfera muñeca → el codo.** Se definen dos esferas
   duales, una centrada en el hombro con radio = longitud del brazo
   superior, otra centrada en el objetivo (muñeca) con radio = longitud del
   antebrazo. Su intersección con el "plano de giro del codo" da un par de
   puntos: las dos soluciones posibles para la posición del codo.
   ```
   σ_hombro = o - ½λ₁²∞
   σ_muñeca = p - ½λ₂²∞
   codo = (σ_hombro ∧ σ_muñeca) ⌟ Π_tilt
   ```
   Si la intersección da un "par de puntos imaginario" (test simple: signo
   de `codo²`), el objetivo está fuera de alcance — mismo caso que hoy
   maneja `coppeliasim_ik_adapter.py` cuando `simIK` no converge, pero aquí
   se detecta algebraicamente, sin necesidad de iterar.
2. **Ratio de líneas → rotor del codo/hombro.** El ángulo de cada
   articulación sale de un rotor calculado directamente entre dos líneas
   (dirección actual vs. dirección deseada del eslabón), sin resolver
   ningún sistema no lineal.
3. Solo al final, si hace falta el ángulo escalar para mandarlo al
   controlador articular, se saca con un logaritmo — la trigonometría
   aparece una vez, no en cada paso de iteración.

El propio libro cita que esta técnica fue **~40% más rápida** que la
solución clásica basada en ángulos para el robot de su ejemplo. No es una
promesa vacía de "CGA es elegante": es una alternativa concreta a Jacobiano
iterativo, específicamente aplicable si el CR5 tiene muñeca esférica (dato
a confirmar en su ficha técnica antes de comprometerse a este método).

## 6. Interpolación de movimiento rígido (mejora directa sobre `Trajectory`)

`Trajectory.straight_line` hoy interpola linealmente en **espacio de
articulaciones** (ángulo a ángulo), lo cual es lo más simple posible pero
no corresponde a una línea recta cartesiana real del efector. CGA da un
logaritmo cerrado de un movimiento rígido (traslación+rotación combinadas),
que permite interpolar el movimiento del **efector** de forma natural:

```
V^(1/N) = exp( log(M) / N )
```

Aplicar `V^(1/N)` `N` veces interpola (y extrapola) el movimiento de forma
suave, con la rotación y la traslación acopladas correctamente (a diferencia
de interpolar posición y orientación por separado, que es lo habitual en
soluciones clásicas). Es una candidata natural para `PlanningPort`
(Bloque 4) cuando haga falta trayectorias cartesianas suaves, no solo
rectas en articulaciones.

## 7. Ajuste de esfera a puntos — directamente para `PerceptionPort` (Bloque 3)

El libro da un método práctico para ajustar un `SphereObstacle` a una nube
de puntos detectada (el caso de uso: la cámara ve un objeto, hay que
aproximarlo con una esfera para el planificador). Resumen del método:

- Cada punto `p_i` se codifica como el vector `[1, p_x, p_y, p_z, ½(p_x²+p_y²+p_z²)]`.
- Se arma la matriz `D = Σ [[p_i]]·[[p_i]]ᵀ·[[M]]` (`M` = la métrica del
  modelo conforme, dada explícitamente en el libro).
- El vector `σ` (la esfera dual buscada) es el vector singular de `D`
  correspondiente al valor singular más pequeño (SVD estándar).
- El centro y el radio de la esfera se leen directamente de las componentes
  de `σ` (tabla 14.1 del libro).

Es una sola SVD, sin iteración, y da directamente los parámetros que
`SphereObstacle.__init__` ya espera (`center`, `radius`). Buen candidato
concreto para la primera versión de `PerceptionPort` en simulación
(Bloque 3): ground-truth de CoppeliaSim → nube de puntos sintética →
`SphereObstacle` vía este ajuste, antes incluso de tener visión real.

## 8. Lo que esto NO resuelve todavía

- Esto es la matemática, no la implementación: `pygafro`/`gafro_ros` siguen
  sin compilar para el CR5, que sigue siendo el bloqueo real de
  `GaKinematicsAdapter` (ver su propio TODO). Esta nota reduce el riesgo de
  no entender el álgebra cuando llegue ese momento, no sustituye el trabajo
  de integración.
- El ejemplo de IK geométrica del libro es de 2 eslabones; extenderlo a los
  6 ejes reales del CR5 es trabajo adicional de verdad, aunque la técnica
  (esferas + ratios de líneas) se generaliza razonablemente bien a brazos
  con muñeca esférica.
- No cubre nada de percepción real (visión, detección) — solo el ajuste
  geométrico una vez que ya hay una nube de puntos candidata.

## Referencias de página (para volver a consultar sin releer todo)

Todas dentro de *Geometric Algebra for Computer Science* (Dorst, Fontijne,
Mann, 2007):

- Cap. 13, *The Conformal Model*: pp. 356–396. Puntos/planos/esferas como
  vectores: §13.1 (pp. 356–364). Versores euclídeos: §13.2 (pp. 364–369).
  Planos y direcciones: §13.3 (pp. 370–378). Movimientos rígidos y
  logaritmo: §13.5 (pp. 379–384). Interpolación: §13.6 (p. 385).
- Cap. 14, *New Primitives for Euclidean Geometry*: Rounds (puntos, círculos,
  esferas como blades): §14.1 (pp. 398–403). Ajuste de esfera a puntos:
  §14.5 (pp. 417–420). **Cinemática (directa e inversa)**: §14.6
  (pp. 420–426).
