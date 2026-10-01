---
tags: [arquitectura]
---

# Scene y Percepción

`Scene` (`geometry_kernel/scene.py`) no es un puerto, es el **agregado** que
representa lo que el sistema sabe de la escena en un instante dado: planos,
obstáculos y objetos, todos con nombre. La produce percepción
(`PerceptionPort`, ver [[Puertos y Adaptadores]]) y la consumen planificación
y (en el futuro) grounding.

```python
Scene(
    planes: Dict[str, Plane],
    obstacles: Dict[str, SphereObstacle],
    objects: Dict[str, Point],
    bodies: Dict[str, Body],      # desde el 30/09
)
```

**`bodies` (30/09)**: cuerpos sólidos con forma (`Box`, `Cylinder`,
`Sphere`), pose del centro, `graspable` y color opcional
(`geometry_kernel/bodies.py`). Es lo que se dibuja en CoppeliaSim y lo que
la pinza puede coger. `Body.bounding_sphere()` da la esfera envolvente,
para los planificadores que solo saben de esferas. Va en un campo aparte
de `objects` a propósito: `objects` son puntos con nombre y lleva el
objetivo del movimiento (`"objetivo"`) por ROS hasta `Commander`; cambiarle
el tipo rompía ese camino. `bodies` todavía no viaja por ROS. Se describe
en YAML en [[Células y Escenarios]].

Inmutable a propósito, como el resto de value objects del dominio: cada
`with_plane`/`with_obstacle`/`with_object`/`merge` devuelve una `Scene`
nueva.

## Por qué dict y no lista

`obstacles` (y `planes`/`objects`) son `Dict[str, X]`, no listas — decisión
tomada al diseñar `perception_node`. Con nombre estable como clave, un
obstáculo reportado dos veces con el mismo nombre se **actualiza** en vez de
duplicarse, y un productor externo (fichero, ground truth de CoppeliaSim...)
puede releer su fuente entera en cada ciclo sin llevar diff. Ver
[[Decisiones de Diseño Clave]].

## `Scene.merge` — cómo Commander ensambla la escena completa

`scene_a.merge(scene_b)` combina clave a clave (en un choque, gana el
segundo) — pensado para que `Commander` combine las piezas parciales que le
reporten uno o más perceptores en una única `Scene` completa, sin que
`controller_node` tenga que escuchar directamente a un único productor (ese
era el boceto anterior). Hoy `Commander.follow_perception(session_name)` se
suscribe a `/perception/scene` (topic **global**, sin namespace de sesión —
vive fuera de cualquier `ControlSession`) y reenvía como `send_goal`
cualquier objetivo nuevo en `Scene.objects["objetivo"]`.

`/perception/scene` viaja como JSON dentro de `std_msgs/String` (mismo
patrón que `<ns>/feedback`, sin paquete de interfaces `.msg` propio) —
`to_scene_msg`/`from_scene_msg` en `ros2_kit/messages.py`.

## RobotDescription — el robot, no la escena

`shared_kernel/robot_description.py` describe la geometría de un manipulador
serie concreto: la cadena de `JointDescription` (nombre, tipo, origen en
configuración home, eje) entre `base_link` y `tip_link`. Es la única fuente
de la que `PoeKinematicsAdapter` (y, cuando exista, el adaptador GA) derivan
sus propios twists/bivectores — **los mismos números interpretados en dos
álgebras distintas**, no una conversión con pérdida. `urdf_kit.parse_urdf_file`
lo construye a partir de un `.urdf` real, plegando cualquier `<joint
type="fixed">` intermedio en el origen de la articulación móvil siguiente.

Esto es lo que permite que [[Commander y ControlSession|ControlSession]]
derive `joint_names` de un robot arbitrario en vez de tenerlos hardcodeados
para el CR5 — pieza central del Bloque 9, ver [[Estado del Roadmap]].
Referencia función por función de `parser.py` y `robot_description.py`:
[[urdf_kit y RobotDescription]].

## Ver también

- [[Puertos y Adaptadores]]
- [[Commander y ControlSession]]
- [[Arquitectura Hexagonal]]
- [[Primitivas Geométricas]] — referencia función por función de `Pose`/`Point`/`Plane`/`SphereObstacle` y de los métodos de `Scene`
- [[urdf_kit y RobotDescription]] — referencia función por función de cómo se deriva `RobotDescription` de un `.urdf`
