---
tags: [arquitectura]
---

# Value Objects y Dominio

Referencia función por función de los cuatro archivos de `shared_kernel`
que forman el vocabulario de dominio puro: ninguno depende de ROS2, de
CoppeliaSim ni de ninguna estrategia de cinemática concreta. Ver
[[Arquitectura Hexagonal]] para dónde encaja `shared_kernel` en la
dirección de dependencias.

## `either.py` — `Either`, para fallos sin excepciones

`Left`/`Right` (ambos `@dataclass(frozen=True)`, genéricos) son las dos
mitades de `Either[L, A] = Union[Left[L, A], Right[L, A]]`. `left(value)`/
`right(value)` son azúcar para construirlos sin importar las clases
directamente. `is_left()`/`is_right()` son el único mecanismo de
discriminación — no hay pattern matching estructural en el resto del
código, todo comprueba con estos dos métodos.

Se usa en los constructores `.create(...)` de `value_objects.py` y
`trajectory.py`: en vez de lanzar una excepción ante una entrada
inválida, devuelven un `Left` con el error de dominio correspondiente. La
excepción sigue existiendo (es el `value` del `Left`), pero envuelta —
quien llama decide si la relanza o la maneja como dato.

## `errors.py` — errores de dominio, sin depender de ningún framework

Cinco excepciones simples, casi todas sin lógica propia:
- `InvalidJointPositionError` — nombre vacío o ángulo no finito (`NaN`/`inf`) en `JointPosition.create`.
- `EmptyJointConfigurationError` / `EmptyTrajectoryError` — mensaje fijo en el constructor; ambas protegen el mismo invariante ("una secuencia no puede estar vacía") en dos niveles distintos (una configuración de articulaciones, una trayectoria de configuraciones).
- `TrajectoryVerificationFailedError` — declarada aquí pero no lanzada por ningún archivo de este clúster; su emisor vive en otro paquete (probablemente `controller_node`, a verificar si se documenta ese código).
- `InvalidRobotDescriptionError` — para `urdf_kit`/`robot_description.py`, ver [[urdf_kit y RobotDescription]] si existe.

Viven junto al `Either`, no dentro de una clase concreta, para que
cualquier capa pueda capturarlas por tipo sin conocer quién las lanzó —
mismo criterio que `RobotConnectorError` en [[RobotConnectorPort]].

## `value_objects.py` — el vocabulario común a todos los adaptadores

**`JointPosition`** — `(joint_name: str, angle_radians: float)`, frozen.
`JointPosition.create(...)` valida dos cosas antes de dejar construir el
objeto: nombre no vacío/no solo espacios, y ángulo finito (rechaza
`NaN`/`inf` explícitamente con `math.isfinite`, no solo "no es None") —
devuelve `Either[InvalidJointPositionError, JointPosition]`.

**`JointConfiguration`** — un waypoint completo: lista de
`JointPosition`. `.create(...)` rechaza lista vacía
(`EmptyJointConfigurationError`). `.angle_of(joint_name)` busca linealmente
por nombre y lanza `KeyError` (no `Either`) si no existe — inconsistencia
deliberada o no, es la única búsqueda por nombre que no pasa por el canal
`Either`; conviene recordarlo si se depura un `KeyError` inesperado aguas
arriba.

> Ambas clases son `@dataclass(frozen=True)`: cualquier "modificación"
> (p. ej. una trayectoria interpolada) construye objetos nuevos en vez de
> mutar — mismo patrón que `Scene` en [[Primitivas Geométricas]].

## `trajectory.py` — la salida de todo `KinematicsPort`

**`Trajectory`** — `waypoints: List[JointConfiguration]`, frozen. Es lo
que produce cualquier adaptador de [[KinematicsPort]]/[[PlanningPort]] (PoE,
GA, DH, `simIK`...) y lo que consume, waypoint a waypoint, un
`RobotConnectorPort` real.

- `Trajectory.create(waypoints)` — rechaza lista vacía
  (`EmptyTrajectoryError`), vía `Either` otra vez.
- `Trajectory.straight_line(start, end, steps)` — interpola LINEALMENTE
  en espacio de articulaciones (ángulo a ángulo, no en cartesiano — una
  "recta" en ángulos no es una recta en el espacio de trabajo). Construye
  `steps + 1` waypoints con `t = i/steps` de `start` a `end` inclusive.
  Pensado para que cualquier `KinematicsPort` que ya tenga una
  `JointConfiguration` objetivo (IK resuelta por el método que sea) pueda
  apoyarse en esto en vez de reimplementar la interpolación — lo usa al
  menos `StraightLineKinematicsAdapter`.
  El comentario del propio código señala la invariante: `steps >= 1`
  garantiza al menos `[start, end]`, por lo que `Trajectory.create(...)`
  nunca puede fallar aquí — de ahí que el código haga `.value` directo
  sobre el `Either` sin comprobar `is_left()` primero.

## Ver también

- [[Primitivas Geométricas]]
- [[Arquitectura Hexagonal]]
- [[KinematicsPort]]
- [[Scene y Percepción]]
