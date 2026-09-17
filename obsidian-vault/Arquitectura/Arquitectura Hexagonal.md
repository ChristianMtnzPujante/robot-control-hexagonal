---
tags: [arquitectura]
---

# Arquitectura Hexagonal

Lenguaje común del proyecto: `Comandante -> Trajectory -> set_joints`. Un
**puerto** define una capacidad (`typing.Protocol`, no `ABC` — un adaptador no
hereda de nada, solo implementa los métodos con la forma correcta; duck
typing verificado estáticamente por mypy). Un **adaptador** implementa un
puerto contra algo concreto (CoppeliaSim, el CR5 físico, una estrategia de
cinemática). El dominio nunca conoce los adaptadores, solo los puertos — ver
[[Puertos y Adaptadores]].

## Paquetes y dirección de dependencias

```
geometry_kernel   ← shared_kernel  ← ros2_kit ← {robot_node, controller_node, perception_node, commander}
(primitivas puras)  (dominio ejec.)   (infra ROS2)   (paquetes ROS2 reales)
```

- **`geometry_kernel`** — nivel más bajo: `Pose`, `Point`, `Plane`,
  `SphereObstacle`, `Scene`. No depende de nada, ni siquiera de
  `shared_kernel` — es al revés. Ver [[Scene y Percepción]] y, para la
  referencia función por función, [[Primitivas Geométricas]].
- **`shared_kernel`** — value objects de ejecución/cinemática
  (`JointConfiguration`, `Trajectory`, `Either`), los puertos, y
  `RobotDescription`. Depende de `geometry_kernel` y reexporta
  `Pose`/`Scene` para que el resto siga haciendo
  `from shared_kernel import ...` sin enterarse. Referencia función por
  función: [[Value Objects y Dominio]].
- **`ros2_kit`** — infraestructura ROS2 compartida: mensajes ↔ dominio
  (`ros2_kit/messages.py`), QoS (`ros2_kit/qos.py`), configuración
  declarativa por YAML (`ros2_kit/node_config.py`). `shared_kernel` sigue sin
  depender de esto ni de ROS2 en absoluto. Referencia función por función:
  [[Infraestructura ROS2 (ros2_kit)]].
- **`ros1_kit`** — boceto deliberadamente sin conectar a nada (pensado para
  un puente `ros1_bridge` hacia el driver oficial del CR5). Descartado el
  04/09 a favor de reimplementar TCP/IP directo — ver
  [[Decisiones de Diseño Clave]] y, para el detalle del boceto en sí,
  [[Infraestructura ROS2 (ros2_kit)]].
- **`urdf_kit`** — parsea un `.urdf` real y produce un `RobotDescription`
  (cadena de articulaciones entre `base_link` y `tip_link`). Es lo que
  permite que `ControlSession` derive `joint_names` de un robot arbitrario en
  vez de tenerlos hardcodeados — ver Bloque 9 en [[Estado del Roadmap]] y la
  referencia función por función en [[urdf_kit y RobotDescription]].
- **`robot_node` / `controller_node` / `perception_node` / `commander`** —
  los cuatro paquetes ROS2 reales. `robot_node` envuelve
  `RobotConnectorPort`, `controller_node` envuelve
  `KinematicsPort`/`PlanningPort`/`PlannerSelectionPort`, `perception_node`
  envuelve `PerceptionPort`, `commander` contiene `Commander` y
  `ControlSession` — ver [[Commander y ControlSession]].

## Por qué separar `geometry_kernel` de `shared_kernel`

`Scene` la necesita tanto el contexto de ejecución/cinemática como el futuro
contexto de percepción, y percepción no necesita saber nada de
`JointConfiguration` ni de `Trajectory`. Separar el paquete hace ese límite
explícito en vez de implícito.

## Lo que hace el sistema "adaptable" de verdad

Añadir una estrategia de cinemática nueva, o un robot nuevo, es añadir una
clase que implemente el puerto correspondiente — nunca tocar el dominio ni
`Commander`. `robot_node/node.py::_build_adapter` y
`controller_node/node.py::_build_adapter` resuelven qué adaptador construir
vía un registro `str -> factoría`, no un `if/elif` que crece sin límite.

## Ver también

- [[Puertos y Adaptadores]]
- [[Commander y ControlSession]]
- [[Scene y Percepción]]
- [[Estado del Roadmap]]
- [[Value Objects y Dominio]] · [[Primitivas Geométricas]] · [[Infraestructura ROS2 (ros2_kit)]] · [[urdf_kit y RobotDescription]] — referencia función por función de cada paquete de este mapa
