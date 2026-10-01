---
tags: [guia, cr5, real, seguridad]
---

# Prueba controlada contra el CR5 real

Cómo probar contra el robot físico todo lo nuevo de [[Células y Escenarios]]
y [[Roles del Commander]] (célula compilada, `Manipulator`, "siempre desde
arriba", semillas de IK, `SpeedFactor`), por etapas y sin sorpresas.
Script: `ros2 run commander real_cell_check`. Ensayado en CoppeliaSim con
el mismo script y la misma célula el 01/10.

> [!warning] Nada de esto se ha ejecutado todavía contra el robot real
> En particular, `SpeedFactor` no se ha mandado nunca a este controlador.
> Si lo rechaza, la prueba se para ANTES del primer movimiento (se manda
> justo tras `EnableRobot`).

## Antes

1. **Medir y escribir** en `descriptions/scenes/laboratorio_cr5.yaml`:
   - `planes.mesa`: la altura del tablero en el marco de la base (z = 0 si
     el robot está atornillado a la misma mesa).
   - `points.prueba_*`: puntos en el aire para la etapa 3 (más de 0,30 m
     sobre la mesa, sin nada entre ellos).
2. **Comprobar a ojo** que el camino hasta la postura `prueba_segura`
   (`[0, 0, 90, 0, -90, 0]`, brida mirando abajo a ~0,47 m) está despejado
   desde donde esté el robot.
3. **Robot sin energizar** en el teach pendant (el modo TCP solo se concede
   así) y la **seta de emergencia a mano**.
4. **Ensayo en simulación** con el mismo fichero:
   `ros2 run commander real_cell_check --target sim --yes`.

## Lanzarla

```bash
source install/setup.bash
# 1º solo lectura: no mueve nada
ros2 run commander real_cell_check --stages 0
# después, etapa a etapa (o todas: por defecto 0-4)
ros2 run commander real_cell_check --stages 0,1,2 --speed 10
ros2 run commander real_cell_check --stages 0,2,3,4 --speed 10
```

Cada etapa con movimiento: **planifica en seco** (los waypoints exactos que
se mandarían, sin mover nada), **revisa** (salto máximo entre waypoints ≤
`--max-step`, 5°; ni la brida ni el punto de agarre por debajo de
`mesa + --clearance`, 0,10 m), **enseña el resumen y pide "s"**, ejecuta y
**mide** (lee las articulaciones y compara: si el error pasa de 0,5° o
3 mm, se para). Al salir, también con Ctrl+C, cierra la pinza y
des-energiza. Deja un informe JSON (`real_cell_check_<fecha>.json`).

| Etapa | Qué hace | Mueve |
| --- | --- | --- |
| 0 lectura | Articulaciones, brida, modo del robot, pinza, velocidad configurada | Nada |
| 1 pinza | Abrir, cerrar, abrir en el aire (para si dice que sujeta algo) | Dedos |
| 2 postura | A `prueba_segura` en articulaciones (aquí se habilita y se fija `SpeedFactor`) | Brazo |
| 3 puntos | `move_to_position` a cada `prueba_*` | Brazo |
| 4 recta | Baja 5 cm en recta cartesiana y vuelve | Brazo |
| 5 agarre | `pick` + `place`. Solo con `--allow-contact --pick X --place Y`, y pide escribir "medido" sobre `grasp_offset` | Brazo, toca la mesa |

## Qué mirar

- Que el robot vaya **lento** (10 %): si no, `SpeedFactor` no ha hecho efecto.
- El **error medido** de cada movimiento: es lo primero que diremos de la
  cinemática del URDF contra el robot de verdad.
- Si algún plan se para por un PROBLEMA: copiar el resumen; es justo lo que
  la revisión tiene que cazar.

## El hallazgo que motivó la revisión en seco (01/10)

Ensayando un `pick` desde la home en simulación, la IK de PoE convergió a
`joint2 = +309°` y la interpolación habría llevado la herramienta a
z = −0,72 m. La revisión lo paró antes de mover nada. Corregido en
`Manipulator.move_to_pose`: se resuelve desde la postura actual y desde
cada postura conocida, cada solución se lleva a la vuelta más cercana, y se
elige la que menos mueve. Decisión en [[Decisiones de Diseño Clave]].
**La consola (`cell_console`) todavía no tiene esta revisión**: es la
primera pieza del guardián.

## Ver también

- [[Pinza Robotiq 2F - Uso práctico]]
- [[Scripts de Demostración]]
