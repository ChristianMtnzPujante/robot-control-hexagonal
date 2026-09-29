---
tags: [investigacion, cga, llm, objetivo-inicial]
---

# Ontología del Dominio (lenguaje CGA)

> [!todo] (29/09) OBJETIVO INICIAL: redactar el artículo
> Empezó como línea de investigación anotada para no perderla. El mismo
> 29/09 pasa a ser el **objetivo inicial**: un artículo de revisión que
> termina en una propuesta de ontología, con una parte práctica hecha con
> este repo. Plan en [[#Objetivo inicial: el artículo (29/09)]]; decisión
> en [[Decisiones de Diseño Clave]]. Todavía no hay diseño de la
> ontología.

## Objetivo

Definir una **ontología de objetos y relaciones** propia del proyecto: un
lenguaje completo con el que se pueda describir el dominio (robot, escena,
objetos, tareas) y que sirva a la vez a tres consumidores:

1. **Solvers clásicos.** Cada término del lenguaje tiene que poder
   traducirse a algo calculable: residuo + Jacobiano, restricción de
   igualdad o desigualdad, condición de fin.
2. **CGA, sobre todo.** El lenguaje debe recoger lo que CGA permite
   expresar y otros formalismos no: primitivas como objetos algebraicos
   (punto, recta, plano, esfera, círculo), incidencia X ∧ A = 0,
   intersección con el meet, diagnóstico por signos (dentro/fuera,
   atraviesa/tangente), y la misma fórmula M X M̃ para mover cualquiera de
   ellas.
3. **Un LLM.** Un vocabulario común, cerrado y con semántica clara, para
   que el modelo describa tareas en términos geométricos ("el eje de la
   herramienta sobre esta recta hasta tocar este plano") en vez de
   enumerar poses, y que eso se pueda verificar antes de ejecutarlo.

## Cómo encaja con lo que ya hay

- Es la capa semántica que le falta a la decisión abierta del 23/09
  ([[Decisiones de Diseño Clave]]): el planificador emite **tramos de
  primitivas combinadas con condición de fin**, y el ejecutor los resuelve
  con residuo + Jacobiano (`GeometricTaskPort`, provisional). La ontología
  sería el lenguaje de esos tramos.
- Toca la cuestión pendiente (a) de esa decisión, y la de bounded contexts
  de [[Primitivas Geométricas]]: ¿la ontología es neutra (se traduce a CGA
  o a PoE) o es conforme de origen?
- Tesis: F1.4 (tools con esquema CGA para el LLM) y F1.6 (verificar antes
  de actuar). También el Bloque 6 (API de tools) del roadmap.

## Objetivo inicial: el artículo (29/09)

**Por qué aquí.** La ontología está implícita en tres objetivos de la
propuesta de tesis (`~/Desktop/doctorado/propuesta_tesis_CGA_LLM_v2.tex`),
pero ninguno la define:
- H4: "el álgebra conforme como lenguaje común entre las tres capas".
- H2.2: tools MCP "cuyo esquema de parámetros sean entidades CGA".
- H3.1: "definir la escena, los objetos y los movimientos en lenguaje CGA".

Además, H1.1 necesita un banco de tareas descrito de forma neutra para
comparar CGA con SE3 o CasADi de forma justa. Es la pieza que une la
tesis, y hoy la propuesta da por hecho que ese lenguaje existe.

**Forma.** Se fusiona con el hito que ya estaba previsto en la Fase 1 (el
artículo de revisión del estado del arte) en lugar de ser un artículo
suelto. Una ontología propuesta sin validar tiene poco recorrido, y una
revisión que termina en un marco unificado es un formato habitual.
Estructura de trabajo, provisional:

1. **Revisión.** De la especificación de tareas por restricciones entre
   features (TFF/iTaSC/eTaSL, KnowRob, De Laet et al.) a CGA y a los LLM
   (Kamarianakis et al., Hong et al., NARRATE...). Con el protocolo de
   búsqueda sistemática de F1.3. Esta revisión también sirve para
   comprobar si de verdad no existe ya una ontología CGA para robótica.
2. **Propuesta de ontología.** Features, relaciones, tareas (prioridad,
   desigualdad, condición de fin) y su traducción a CGA y a residuo +
   Jacobiano. Vocabulario tipado con semántica definida, más cerca de un
   DSL o de un esquema que de OWL.
3. **Parte práctica: escenas de ejemplo.** Unas cuantas escenas definidas
   con el software de este repo, descritas en la ontología, traducidas a
   CGA y con capturas de CoppeliaSim, que se proponen como banco para la
   siguiente fase (H1.1, Fase 2). Material que ya existe y se puede
   reutilizar:
   - `Scene` y las primitivas de `geometry_kernel` ([[Primitivas Geométricas]]).
   - Los ficheros de escena de `FilePerceptionAdapter` ([[Scene y Percepción]]).
   - `build_cr5_scene` y las demos de CoppeliaSim ([[Scripts de Demostración]]):
     obstáculos, semicírculo, PoE frente a GA.
   - La traducción cartesiano→CGA de `docs/algebra_geometrica_conforme.md`.
   - Las pruebas de incidencia, meet y prioridades de `docs/cga_*.py`.
4. **Validación mínima** (lo que la convierte en aportación y no solo en
   propuesta):
   - Expresividad: las escenas y tareas del punto 3 se pueden describir.
   - Ejecutabilidad: al menos una relación se resuelve de punta a punta
     (ya hay prueba parcial de incidencia y meet).
   - Uso por un LLM: un experimento pequeño, opcional según el alcance.

**Pendiente:**
- Hablarlo con el director, porque cambia el contenido del primer hito.
- Ver en qué se diferencia de Kamarianakis et al. (2026).
- No convertirlo en un trabajo de ingeniería ontológica formal.

## Pistas para empezar (pendientes de leer y verificar)

No parece haber una ontología de CGA para robótica, pero sí una línea con
fundamento que encaja casi uno a uno: **la especificación de tareas como
restricciones entre features geométricas**.

- **Task Frame Formalism / iTaSC** (De Schutter, Bruyninckx et al.): las
  tareas se especifican como relaciones entre features del robot y
  features del objeto.
- **Kresse y Beetz** (grupo de KnowRob): movimientos descritos como
  restricciones entre features de tipo punto, línea y plano. Por ejemplo,
  "el eje de la espátula perpendicular al plano de la sartén". Es CGA sin
  decirlo.
- **De Laet, Bruyninckx et al., "Geometric relations between rigid bodies:
  semantics for standardization"** (IEEE RAM, 2013): una semántica
  rigurosa de poses y marcos, pensada explícitamente como base para
  estandarizar.

## Preguntas que habrá que contestar

- Qué entidades (features) y relaciones forman el núcleo, y cuáles se
  derivan. Por ejemplo, ¿"perpendicular" es primitiva o se deriva de
  ángulo + dirección?
- Cómo se traduce cada relación a CGA y a un residuo para el solver: la
  incidencia ya está probada en `docs/cga_tareas_linea_prioridades.py`.
- Cómo se representan la prioridad, las desigualdades (evitar obstáculos)
  y la condición de fin de un tramo.
- Cómo se serializa para un LLM (esquema de tools, texto controlado) sin
  perder la semántica.
- Qué aporta frente a KnowRob/iTaSC: la hipótesis es que CGA da un álgebra
  única para todas las relaciones, en lugar de un caso distinto por cada
  par de features.

## Ver también

- [[Decisiones de Diseño Clave]] — decisión abierta del 23/09 (tareas CGA
  por primitivas, planificador y ejecutor separados)
- [[Primitivas Geométricas]]
- [[GaKinematicsAdapter]]
- [[Estado del Roadmap]]
- [[2026-09-29]]
