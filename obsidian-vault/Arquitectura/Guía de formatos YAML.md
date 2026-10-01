---
tags: [arquitectura, referencia, generado]
---

# Guía de formatos YAML

> [!warning] Nota GENERADA, no editar a mano
> Sale de los esquemas de `src/commander/commander/cell/` con
> `ros2 run commander cell_guide --write`. Un test falla si se queda
> atrás respecto al código. Para cambiar un campo, cámbialo en su esquema.

Tres clases de YAML que no se mezclan (decisión del 01/10, ver
[[Decisiones de Diseño Clave]]): los **elementos del mundo**, la
**composición** (la célula) y los **tipos de nodo**. Cómo encajan: ver
[[Células y Escenarios]]. «Obligatorio» puede ser una condición (p. ej. «si
`box`»): la comprueba el lector, no la tabla.

## Elementos del mundo

Uno por fichero en `descriptions/`. No saben de ROS: un robot o una escena se reutilizan en varias células.

### Robot

`descriptions/robots/<nombre>.yaml`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `urdf` | ruta | sí |  | URDF del robot. Relativa al propio YAML, o con `~`. |
| `package_prefix` | ruta | sí |  | Directorio que sustituye a `package://` en las mallas del URDF. |
| `base_link` | texto | sí |  | Primer eslabón de la cadena serie (cinemática). |
| `tip_link` | texto | sí |  | Último eslabón de la cadena serie: la brida. |
| `joint_names` | lista de textos | no |  | Joints en orden. Si falta, se derivan del URDF (base_link → tip_link). |
| `postures` | dict nombre → grados | no |  | Posturas propias del robot (p. ej. `home`), en orden de joints. |
| `sim` | sección | no |  | Datos para CoppeliaSim (ver «Robot: sim»). |
| `real` | sección | no |  | Datos para el robot real (ver «Robot: real»). Sin ella, solo simulación. |

### Robot: sim

`robot: sim`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `root_alias` | texto | sí |  | Objeto raíz que crea simURDF al importar (para borrarlo al reconstruir). |
| `tip_object` | texto | no |  | Objeto de CoppeliaSim que hace de punta (rastro de waypoints). |

### Robot: real

`robot: real`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `adapter` | nombre | sí |  | Adaptador de conexión (`adapters.py`), p. ej. `cr5_tcp`. |
| `joint_limits_degrees` | lista de grados | no |  | Límite por joint. Solo puede ESTRECHAR el de fábrica. |

### Herramienta

`descriptions/tools/<nombre>.yaml`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `urdf` | ruta | sí |  | URDF de la herramienta, con su raíz libre para colgarla de una brida. |
| `package_prefix` | ruta | sí |  | Directorio que sustituye a `package://` en sus mallas. |
| `driven_joint` | texto | sí |  | El joint que se manda. Los `<mimic>` que lo siguen se leen del URDF. |
| `mounts` | dict robot → montaje | sí |  | Dónde se monta en cada robot (ver «Herramienta: montaje»). |
| `grasp` | sección | sí |  | Cómo agarra (ver «Herramienta: agarre»). |
| `sim` | sección | no |  | Adaptador en simulación (ver «Herramienta: sim»). |
| `real` | sección | no |  | Adaptador en el robot real (ver «Herramienta: real»). |

### Herramienta: montaje

`tool: mounts.<robot>`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `parent_joint` | texto | sí |  | Joint del robot del que cuelga (la brida). |
| `position` | [x, y, z] m | no | [0, 0, 0] | Desplazamiento respecto a ese joint con el robot a cero (acoplador). |
| `rpy_degrees` | [roll, pitch, yaw] ° | no | [0, 0, 0] | Giro respecto a ese joint. |

### Herramienta: agarre

`tool: grasp`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `offset` | m | sí |  | Distancia de la brida al centro de lo agarrado, según el MODELO. En real, la célula debe dar la medida. |
| `pads` | sección | no |  | Geometría de las yemas para el agarre cinemático en simulación (ver «Herramienta: yemas»). |

### Herramienta: yemas

`tool: grasp.pads`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `frame_joints` | dict | sí |  | `left_knuckle`, `right_knuckle`, `left_tip`, `right_tip`: joints con los que se construye el marco de agarre. |
| `z_range` | [mín, máx] m | sí |  | Altura de las yemas en ese marco (origen entre los nudillos, z hacia las puntas). |
| `half_width` | m | sí |  | Media anchura de las yemas. |
| `half_gap_by_fraction` | lista de m | sí |  | Distancia del plano medio a la cara interior, de abierta (0) a cerrada (1), equiespaciada. |

### Herramienta: sim

`tool: sim`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `adapter` | nombre | sí |  | Adaptador en CoppeliaSim (`adapters.py`), p. ej. `coppeliasim_urdf_gripper`. |

### Herramienta: real

`tool: real`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `adapter` | nombre | sí |  | Adaptador real (`adapters.py`), p. ej. `robotiq_modbus_flange`. |

### Escena

`descriptions/scenes/<nombre>.yaml`

Todo en metros y en el marco de la base del robot (que en simulación es el del mundo).

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `bodies` | dict nombre → cuerpo | no |  | Cuerpos sólidos (ver «Cuerpo»). Van a `Scene.bodies`. |
| `points` | dict nombre → [x, y, z] m | no |  | Puntos con nombre, p. ej. destinos. Van a `Scene.objects`. |
| `obstacles` | dict nombre → obstáculo | no |  | Esferas a evitar (ver «Obstáculo»). Van a `Scene.obstacles`. |
| `planes` | dict nombre → plano | no |  | Planos (ver «Plano»). Van a `Scene.planes`. |

### Cuerpo

`scene: bodies.<nombre>`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `shape` | `box` \| `cylinder` \| `sphere` | sí |  | Forma. |
| `size` | [x, y, z] m | si `box` |  | Medidas totales de la caja. |
| `radius` | m | si `cylinder` o `sphere` |  | Radio. |
| `height` | m | si `cylinder` |  | Altura del cilindro (eje en su z). |
| `position` | [x, y, z] m | sí |  | Centro de la forma. |
| `rpy_degrees` | [roll, pitch, yaw] ° | no | [0, 0, 0] | Orientación, convención URDF (ejes fijos x, y, z). |
| `quaternion` | [qx, qy, qz, qw] | no |  | Orientación como cuaternión (en vez de `rpy_degrees`). |
| `graspable` | bool | no | false | Si la pinza lo puede coger. |
| `color` | [r, g, b] 0..1 | no |  | Solo visual. Sin él: rojo si se puede coger, gris si es fijo. |

### Obstáculo

`scene: obstacles.<nombre>`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `center` | [x, y, z] m | sí |  | Centro de la esfera. |
| `radius` | m | sí |  | Radio. |

### Plano

`scene: planes.<nombre>`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `point` | [x, y, z] m | sí |  | Un punto del plano. |
| `normal` | [x, y, z] | sí |  | Normal (vector, no posición). |

## Composición

La célula referencia a los elementos y dice cómo se usan. Compilarla (`compile_cell`) resuelve las referencias, valida cada pieza y las reglas entre piezas.

### Célula

`descriptions/cells/<nombre>.yaml`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `name` | texto | sí |  | Nombre de la célula. |
| `robot` | sección | sí |  | Qué robot y contra qué destino (ver «Célula: robot»). |
| `tools` | lista | no | [] | Herramientas montadas (ver «Célula: herramienta»). De momento, una. |
| `scene` | sección | no |  | Escena inicial (ver «Célula: escena»). Sin ella, vacía. |
| `kinematics` | `poe` \| `ga` | no | poe | Cinemática, construida del URDF del robot. |
| `simulator` | sección | no |  | CoppeliaSim (ver «Célula: simulador»). |
| `postures` | dict nombre → grados | no |  | Posturas de esta tarea (p. ej. `pre_agarre`). Se suman a las del robot; no pueden repetir nombre. |

### Célula: robot

`cell: robot`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `ref` | nombre o ruta | sí |  | Robot de `descriptions/robots/`. |
| `target` | `sim` \| `real` | no | sim | Dónde está el robot. `--target` lo cambia al lanzar. |
| `host` | IP | si `real` |  | IP del controlador. |
| `initial_posture` | nombre | no | todo a 0 | Postura en la que se crea en simulación. En real no se mueve nada al abrir. |

### Célula: herramienta

`cell: tools[i]`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `ref` | nombre o ruta | sí |  | Herramienta de `descriptions/tools/`. |
| `grasp_offset` | m | si `real` |  | Distancia de agarre MEDIDA en este montaje. Sin ella, la del modelo (solo simulación). |

### Célula: escena

`cell: scene`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `ref` | nombre o ruta | sí |  | Escena de `descriptions/scenes/`. |

### Célula: simulador

`cell: simulator`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `port` | entero | no | 23000 | Puerto ZMQ de CoppeliaSim. |
| `step_pause_seconds` | s | no | 0.04 | Pausa entre waypoints (para ver la animación). |

## Tipos de nodo

La interfaz de cada nodo ROS, en su propio paquete. La usa el grafo de nodos de una célula (siguiente paso).

### Tipo de nodo

`src/<paquete>/config/<nodo>.yaml`

La lógica nunca va aquí: `callback` es solo el NOMBRE del método que atiende el canal.

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `node` | `{name}` | sí |  | Nombre del nodo. |
| `parameters` | dict nombre → parámetro | no |  | Parámetros declarados (ver «Tipo de nodo: parámetro»). |
| `publishers` | lista de topics | no |  | Lo que publica (ver «Tipo de nodo: topic»). |
| `subscriptions` | lista de topics | no |  | A qué se suscribe (ver «Tipo de nodo: topic»). |
| `timers` | lista | no |  | `period_parameter` (nombre de un parámetro) + `callback` (método del nodo). |

### Tipo de nodo: parámetro

`node: parameters.<nombre>`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `type` | `string` \| `int` \| `double` \| `bool` \| `string_array` \| `double_array` | sí |  | Tipo ROS 2 del parámetro. |
| `default` | valor | sí |  | Valor por defecto. |
| `range` | `{min, max}` | no |  | Solo `int`/`double`: rclpy rechaza en ejecución lo que quede fuera. |

### Tipo de nodo: topic

`node: publishers[i] / subscriptions[i]`

| Campo | Tipo | Obligatorio | Por defecto | Qué es |
| --- | --- | --- | --- | --- |
| `topic` | nombre | sí |  | Relativo (`goal`): dentro del namespace de la instancia. Absoluto (`/perception/scene`): global. |
| `message_type` | `paquete/msg/Tipo` | sí |  | Tipo de mensaje, p. ej. `sensor_msgs/msg/JointState`. |
| `qos` | entero o perfil | sí |  | Profundidad de cola (entero) o un perfil de `ros2_kit.qos`: `GOAL_QOS`, `SCENE_QOS`, `STRATEGY_QOS`. |
| `callback` | nombre de método | en suscripciones |  | Método del nodo que atiende los mensajes. |
