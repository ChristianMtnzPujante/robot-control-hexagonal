---
tags: [pruebas, arquitectura]
---

# CR5 vs Panda — prueba de generalización

Complemento de [[Anatomía de un Nodo]]. Tras montar la configuración
declarativa por YAML, la pregunta era: ¿generaliza de verdad a otro robot,
o está secretamente atada al CR5 (6 GDL)? Se buscó un robot real con OTRO
número de articulaciones — un Franka Panda, 7 GDL — y se puso a prueba
cada capa, en vivo, contra CoppeliaSim (no contra hardware físico).

## Montaje

Dos instancias de CoppeliaSim a la vez (puertos 23000/23001), cada una con
su robot importado desde su propio URDF, ejecutando una trayectoria
calculada por [[PoeKinematicsAdapter]] hacia un objetivo cartesiano — sin
ningún código nuevo para el Panda, solo parámetros distintos.

| | CR5 | Franka Panda |
|---|---|---|
| Puerto / joints | 23000 · `joint1..joint6` | 23001 · `panda_joint1..7` |
| Grados de libertad | 6 | 7 |
| Waypoints calculados | 21 | 21 |
| Error final (posición) | 0.0533 mm | 0.0080 mm |

## Lo que generalizó sin tocar código

- **Parseo del URDF** (`urdf_kit.parse_urdf_file`) — deriva 7
  `JointDescription`, no 6, sin ningún cambio.
- **Derivación de `joint_names`** (`ControlSession._resolve_joint_names`)
  desde el URDF, pisando un literal distinto — ver [[Anatomía de un Nodo]]
  §5 (caso real, mismo mecanismo).
- **Dos nodos ROS2 reales** (`controller_node`, `strategy=poe`) calculan y
  entregan la trayectoria por topics, solo con
  `-p urdf_path/base_link/tip_link` distintos.
- **IK de Newton-Raphson** ([[PoeKinematicsAdapter]]) converge igual de
  bien con 7 GDL que con 6 — de hecho más rápido: la redundancia ayuda.

## Lo que NO generalizó — hallazgos reales

> [!bug] `parse_urdf_file` descarta los `fixed` que cuelgan tras la última articulación móvil
> El CR5 nunca lo mostró porque su `tip_link` (`Link6`) no tiene ningún
> `fixed` detrás. El Panda sí: la mano real (`panda_hand`) cuelga de
> `panda_link7` por dos joints `fixed` — y ese desplazamiento se pierde
> entero:
> ```
> FK con tip_link=panda_link7: Pose(x=0.088, y=0.0, z=1.033, ...)
> FK con tip_link=panda_hand:  Pose(x=0.088, y=0.0, z=1.033, ...)  ← IDÉNTICA, no debería serlo
>
> # desplazamiento real, calculado a mano desde el URDF -- se pierde entero:
> panda_joint8     (fixed): +0.107 m en Z
> panda_hand_joint (fixed): -45° de yaw
> ```
> Pendiente de arreglo — archivo: `src/urdf_kit/urdf_kit/parser.py`.

> [!bug] `build_cr5_scene` estaba hardcodeada al CR5 de verdad
> Ruta del URDF, prefijo del paquete de mallas y nombres de joint, los
> tres como constantes de módulo — la propia función se llamaba así.
> **Arreglado en el momento**: se extrajo una `build_scene(...)` genérica
> (parámetros en vez de constantes); `build_cr5_scene` quedó como
> envoltorio, sin romper ningún demo existente. Archivo:
> `src/commander/commander/coppeliasim_scene_builder.py`.

## Metodología: un hallazgo que no lo era

Al mirar los dos robots moviéndose, pareció que la ÚLTIMA articulación
(joint6 del CR5, joint7 del Panda) nunca se movía — sospechoso, dos
robots distintos, mismo patrón. Se verificó con una prueba, no con una
suposición:

- **Observación**: en ambas demos, la última articulación termina en
  0.0000 rad — exactamente igual que al empezar.
- **Hipótesis**: ¿hay algo en el código que trata "la última articulación"
  como especial? (grave si fuera cierto — revisado
  `_accumulate_joint_frames`: recorre las articulaciones genéricamente,
  sin indexar ninguna posición.)
- **Contraprueba**: mismo objetivo, pero con la orientación girada 30°
  extra sobre el eje de aproximación — si la articulación final fuera
  "especial", seguiría sin moverse.
- **Resultado**: `joint6` del CR5 → **-0.1496 rad**; `joint7` del Panda →
  **6.5586 rad** — las dos se mueven en cuanto el objetivo lo exige.
- **Conclusión**: no era un hardcodeo — los dos objetivos de prueba
  mantenían la MISMA orientación que la postura inicial, y la última
  articulación de ambos robots es la de giro de muñeca: sin giro que
  corregir, no hacía falta moverla.

Segunda vuelta de la misma sesión: "el objetivo parece estar más cerca de
`link6` que de `link7` al clicar en CoppeliaSim". También verificado con
las poses reales, no a ojo:

```
goal:                  (0.188, 0.0, 1.033)
pose de panda_joint6:  (0.100, 0.0, 1.033)  -- a 8.8 cm del goal
pose de panda_joint7:  (0.188, 0.0, 1.033)  -- a 0.03 mm del goal
```

Cinemáticamente el objetivo está encima de `joint7`, no de `joint6` — la
confusión visual venía de que la MALLA de `link6` (el "hombro" de la
muñeca) se extiende físicamente más allá de su propio pivote, solapando
con el espacio de `joint7`. Clicar sobre geometría visual y preguntar por
el frame cinemático no son la misma pregunta.

## Ver también

- [[Anatomía de un Nodo]]
- [[PoeKinematicsAdapter]]
- [[Conectar un Robot Nuevo]]
- [[Herramientas de CoppeliaSim]] — referencia función por función de `build_scene`, la generalización de `build_cr5_scene` encontrada aquí
- [[urdf_kit y RobotDescription]] — referencia función por función de `parse_urdf_file`, donde vive el bug de los `fixed` tras el tip
