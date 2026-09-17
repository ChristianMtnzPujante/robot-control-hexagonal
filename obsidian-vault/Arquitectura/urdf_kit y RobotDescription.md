---
tags: [arquitectura]
---

# urdf_kit y RobotDescription

Referencia función por función de cómo un `.urdf` real se convierte en un
`RobotDescription` — la pieza central del Bloque 9 (generalización a
robot arbitrario). Ver [[Scene y Percepción]] §"RobotDescription" para el
porqué conceptual (mismos números, dos álgebras) y
[[Conectar un Robot Nuevo]] para el guion práctico de alta de un robot
nuevo.

## `shared_kernel/robot_description.py` — el descriptor, crudo a propósito

- **`JointDescription`** — una articulación móvil en configuración home:
  `name`, `joint_type` (`revolute`/`continuous`/`prismatic`), `origin_xyz`,
  `origin_rpy`, `axis`. El origen ya viene **plegado** respecto de la
  articulación móvil anterior — cualquier `<joint type="fixed">`
  intermedio del URDF ya está compuesto en este origen antes de llegar
  aquí (lo hace `urdf_kit.parser`, no esta clase).
- **`RobotDescription`** — la cadena completa de `JointDescription` entre
  `base_link` y `tip_link`.
  - `RobotDescription.create(joints, base_link, tip_link) -> Either[...]`
    — único constructor válido; valida que haya al menos una
    articulación, que `base_link`/`tip_link` no estén vacíos, que ningún
    nombre de joint esté vacío o duplicado, y que ningún eje sea el
    vector nulo (norma < 1e-9). Cualquier violación vuelve como `left`
    con `InvalidRobotDescriptionError`, nunca una excepción directa — el
    llamador decide qué hacer con el error (ver
    [[Value Objects y Dominio]] para el patrón `Either` en general).
  - `joint_names` / `degrees_of_freedom` — propiedades derivadas, sin
    estado propio.

Se guardan los parámetros geométricos **crudos** (origen + eje por
articulación), no ya convertidos a twists — la misma fuente de la que PoE
y (cuando exista) GA derivan cada uno su propia representación, en su
propia álgebra. Convertirlo antes de tiempo perdería esa dualidad.

## `urdf_kit/parser.py` — de XML a `RobotDescription`

- `parse_urdf_file(path, base_link, tip_link)` / `parse_urdf_string(xml_text, base_link, tip_link)` —
  puntos de entrada; el segundo hace el trabajo real, el primero solo lee
  el fichero. Delegan el parseo XML en `urdf_parser_py.URDF` y llaman a
  `RobotDescription.create(...)`, relanzando como `ValueError` si viene
  `left`.
- `_serial_chain(robot, base_link, tip_link) -> List[str]` — nombres de
  joint (móviles y fijos) de `base_link` a `tip_link`, en orden, vía
  `URDF.get_chain(..., fixed=True)`. **Gotcha real**: `get_chain` solo
  recorre ancestros de `tip_link` (cada link tiene un único padre) — si
  `base_link` no es ancestro de `tip_link` (incluida la dirección
  invertida, o un link que no existe), lanza `KeyError`, que aquí se
  envuelve en `UnsupportedUrdfChainError` con un mensaje explícito en vez
  de dejar escapar el `KeyError` crudo.
- `_fold_fixed_joints(robot, joint_names) -> List[JointDescription]` — el
  núcleo del módulo: acumula una matriz homogénea 4×4 (`accumulated`)
  mientras recorre la cadena; cada `<joint type="fixed">` solo compone su
  `origin` en `accumulated` y continúa (`continue`, sin emitir
  `JointDescription`); al llegar a un joint móvil, vuelca `accumulated`
  ya plegada como el origen de ESE joint y reinicia el acumulador a
  identidad. Si el tipo no es móvil ni `fixed`
  (`floating`/`planar`/desconocido), lanza `UnsupportedUrdfChainError` —
  el CR5 nunca ejercita esta rama porque no tiene fixed joints
  intermedios (por eso `poe_adapter.py._JOINT_ORIGINS` pudo asumirlos a
  mano sin este plegado).
- `_joint_origin(joint)` — defaults del propio formato URDF: `<origin>`
  ausente ⇒ identidad `(0,0,0)`/`(0,0,0)`. Ojo con `_normalize`/el eje:
  si `joint.axis` es `None` el default es **`(1,0,0)`**, no `(0,0,1)`
  como cabría esperar — confirmado contra el comportamiento real de
  `urdf_parser_py`, no documentado a la primera lectura.
- `_normalize(vector)` — normaliza el eje; si la norma es < 1e-9 devuelve
  el vector sin normalizar tal cual (vector nulo), dejando que sea
  `RobotDescription.create` quien lo rechace formalmente — este módulo no
  duplica esa validación.
- `_rpy_to_matrix` / `_matrix_to_rpy` — conversión RPY ↔ matriz de
  rotación, convención de ejes fijos `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
  Duplicado deliberadamente respecto de
  `poe_adapter.py._rpy_to_matrix` — `urdf_kit` no debe depender de
  `controller_node`. `_matrix_to_rpy` resuelve el caso degenerado
  (gimbal lock, `|cos(pitch)| ~ 0`) fijando `roll = 0`.
- `_origin_to_transform` / `_transform_to_origin` — construyen/deshacen
  la matriz 4×4 a partir de `(xyz, rpy)`, usadas por `_fold_fixed_joints`
  para acumular y luego extraer el origen plegado.

## Ver también

- [[Scene y Percepción]]
- [[Conectar un Robot Nuevo]]
- [[Value Objects y Dominio]]
- [[Estado del Roadmap]]
