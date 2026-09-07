---
tags: [pruebas, arquitectura]
---

# Evitación de colisiones en el CR5 — prueba

Dos planificadores de evitación de obstáculos, intercambiables tras
[[PlanningPort]], construidos como experimento práctico sobre la
arquitectura hexagonal existente — con los hallazgos técnicos reales que
salieron por el camino. (1 sept. 2026, rama
`experimento/planificador-evita-obstaculo` → `main`, fusionada con 36
tests en verde.)

En una frase: se implementaron dos planificadores intercambiables (mismo
puerto, distinta estrategia) para que el CR5 evite un obstáculo — uno solo
protege la punta del brazo, el otro protege el brazo entero — y se
verificaron los dos contra CoppeliaSim, no solo en teoría.

## Qué se construyó

| | Planificador A — [[ObstacleAvoidingPlanningAdapter]] | Planificador B — [[WholeBodyObstacleAvoidingPlanningAdapter]] |
|---|---|---|
| Estrategia | Evitación del efector: si la línea recta al objetivo invade un obstáculo, calcula un único punto de paso que lo rodea y deja que la IK resuelva el resto. | Mismo mecanismo, pero comprueba CADA eslabón del brazo (no solo la punta) en cada punto de la trayectoria, e itera el desvío con más margen si algún tramo sigue invadiendo. |
| Trayectoria de prueba | 21 waypoints | 41 waypoints |

La diferencia entre 21 y 41 waypoints no es ruido: es la firma numérica de
que el segundo planificador detectó un problema (el antebrazo invadiendo
el obstáculo) que el primero, mirando solo la punta, ni siquiera vio.

## Hallazgos técnicos por el camino

> [!bug] Hallazgo 1 — la escena se generaba a mano y arrastraba un desajuste de marcos
> El punto de partida era un archivo de escena de CoppeliaSim construido
> manualmente, cuya postura de reposo no coincidía con la configuración
> "cero" del modelo matemático del robot — y cuyo origen no coincidía con
> el marco de referencia interno de la cinemática.
> **Solución**: construir la escena por código, importando el robot
> directamente desde su URDF real. Efecto colateral bueno: el desajuste
> de marcos desaparece por construcción, sin calibración.

> [!bug] Hallazgo 2 — el robot importado quedaba en modo dinámico, no cinemático
> Al importar desde URDF, las articulaciones se configuraban con física
> real activada. El brazo se apartaba de la trayectoria calculada en
> cuanto pasaba tiempo de simulación entre movimientos — el plan era
> correcto, la ejecución no.
> **Solución**: forzar explícitamente el modo cinemático (posición
> controlada directamente, sin física) tras la importación.

> [!bug] Hallazgo 3 — una solución "correcta" de IK puede ser físicamente irrealizable
> El método numérico de cinemática inversa puede converger a un ángulo que
> reproduce la pose deseada pero corresponde a más de una vuelta completa
> de una articulación (p. ej. 540° en vez de 180°) — matemáticamente
> equivalente, físicamente imposible. El simulador recortaba ese ángulo en
> silencio, sin ningún error, dejando el brazo en una postura distinta a
> la calculada.
> **Solución**: comprobación explícita que rechaza cualquier solución
> fuera de ±360° y prueba otra alternativa — mismo tratamiento que si la
> IK no hubiera convergido. Es el mismo tipo de defensa en profundidad que
> luego se generalizó a los límites articulares del CR5 real — ver
> [[Cr5RealRobotAdapter]] y [[Decisiones de Diseño Clave]].

## Una decisión de diseño, con nombre propio

Al discutir cómo debería representarse la geometría del robot de cara a
una futura segunda formulación matemática (álgebra geométrica conforme),
surgió la pregunta de *bounded contexts* (DDD): ambas formulaciones deben
modelarse como contextos independientes, cada uno con su propio lenguaje,
traducidos explícitamente entre sí cuando haga falta — no una única
representación compartida cuyo interior se intercambia según el álgebra.
Se corrigió una decisión previa, ya escrita en el código, que asumía lo
contrario. Ver [[Decisiones de Diseño Clave]], "PoE y CGA son bounded
contexts separados".

## Cómo ejecutar y comparar en vivo

Dos instancias de CoppeliaSim lado a lado, mismo obstáculo, una resuelta
por cada planificador:

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/robot-control-hexagonal
source install/setup.bash
ros2 run commander avoid_obstacle_demo_compare
```

Arranca las dos simulaciones (puertos 23000 y 23001); cada una tarda 1-2
minutos en inicializarse e importar el robot. La terminal imprime el
hallazgo antes incluso de mirar las ventanas:

```
=== Sesión A (puerto 23000): planificador tip-only ===
Trayectoria calculada: 21 waypoints (evitando 1 obstáculo(s))

=== Sesión B (puerto 23001): planificador de cuerpo completo ===
Trayectoria calculada: 41 waypoints (evitando 1 obstáculo(s))
```

Código de color igual en ambas escenas: rojo = objetivo, naranja =
obstáculo, azul = rastro de waypoints. En el puerto 23000 el rastro pasa
en línea recta cerca de la esfera — el planificador simple no la "vio"; en
el 23001 se curva visiblemente al rodearla. En ambas, el último punto
coincide con el objetivo — solo cambia el camino.

Si algo no arranca bien: cerrar ambas ventanas de CoppeliaSim y sus
procesos (`pkill -f coppeliaSim`) antes de relanzar — reutilizar una
instancia ya usada en la misma sesión de terminal es la causa más
habitual de que se quede colgado.

## Próximos pasos anotados en su momento

- **Pseudo-perceptor** — una fuente de "percepción" programática (sin
  cámara todavía) que anuncie un obstáculo/objetivo nuevo mientras el
  sistema corre, primer paso hacia replanificación reactiva real. **Ya
  implementado** — ver [[PseudoPerceptionAdapter]].
- **Auto-descripción de puertos de percepción** — cada fuente de
  percepción debería declarar qué tipo de información ofrece, al estilo
  de cómo una tool de MCP declara su propio esquema. Pensado para cuando
  llegue la integración con un LLM (Bloque 6).

## Ver también

- [[ObstacleAvoidingPlanningAdapter]]
- [[WholeBodyObstacleAvoidingPlanningAdapter]]
- [[PlanningPort]]
- [[Decisiones de Diseño Clave]]
