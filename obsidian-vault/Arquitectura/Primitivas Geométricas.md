---
tags: [arquitectura]
---

# Primitivas Geométricas

Referencia de `geometry_kernel/primitives.py` — el nivel más bajo del
sistema, sin dependencias ni siquiera de `shared_kernel` (ver
[[Arquitectura Hexagonal]]). Todas las clases son `@dataclass(frozen=True)`.

## Representación cartesiana, no CGA — decisión explícita

El propio módulo lo deja escrito como decisión revisada: estas primitivas
usan geometría cartesiana clásica (la que consumen PoE/DH), no álgebra
geométrica conforme (CGA/multivectores). Cuando aterrice el adaptador GA
(Bloque 1), **no** se reescribirán estas clases por debajo con
multivectores — PoE y CGA son bounded contexts distintos, cada uno con su
propio lenguaje geométrico, y forzar un modelo compartido sería acoplar
dos álgebras que no tienen por qué compartir representación. La solución
prevista es un futuro paquete de primitivas conformes con sus propios
tipos, y una traducción explícita en el punto donde se construye la
escena (tabla de traducción ya en `docs/algebra_geometrica_conforme.md`
§2). Ver también [[Decisiones de Diseño Clave]].

## Las cuatro primitivas

- **`Pose`** — `(x, y, z, qx=0, qy=0, qz=0, qw=1)`. Un objetivo cartesiano:
  posición + orientación como cuaternión, con identidad como default. Es
  el tipo que viaja como `goal` en [[KinematicsPort]]/[[PlanningPort]].
- **`Point`** — `(x, y, z)`. Un punto sin orientación — centro de un
  obstáculo, punto de referencia de un plano, posición de un objeto
  detectado. Se reutiliza para varios roles en vez de tener un tipo por
  uso.
- **`Plane`** — `(point: Point, normal: Point)`. `normal` reutiliza el
  tipo `Point` pero como vector, no como posición — no hay un tipo
  `Vector` separado en este módulo, la distinción es solo semántica/de
  nombre de campo.
- **`SphereObstacle`** — `(center: Point, radius: float)`. La primitiva de
  evitación más simple posible: una esfera. `__post_init__` valida
  `radius > 0` y lanza `ValueError` si no (única primitiva con validación
  propia — las otras tres aceptan cualquier float). Suficiente para un
  CHOMP/RRT mínimo (Bloque 4); un objeto real puede aproximarse con una o
  varias esferas.

## Referencia de métodos de `Scene`

`Scene` vive en `geometry_kernel/scene.py`, construida sobre estas cuatro
primitivas (`planes: Dict[str, Plane]`, `obstacles: Dict[str,
SphereObstacle]`, `objects: Dict[str, Point]`). El porqué de `Scene`, de
`Dict` en vez de lista, y de `merge` como mecanismo de ensamblado de
`Commander`, ya está desarrollado en [[Scene y Percepción]] — aquí solo la
lista de métodos, como referencia rápida:

- `Scene.empty()` — una escena vacía (los tres dicts vacíos).
- `.with_plane(name, plane)` / `.with_obstacle(name, obstacle)` /
  `.with_object(name, position)` — cada uno devuelve una `Scene` nueva con
  esa entrada añadida/sobrescrita por clave; no mutan `self`.
- `.merge(other)` — combina dos `Scene` clave a clave; en un choque de
  claves gana `other`. Detalle de uso real en [[Scene y Percepción]].

## Ver también

- [[Scene y Percepción]]
- [[Value Objects y Dominio]]
- [[Arquitectura Hexagonal]]
- [[Decisiones de Diseño Clave]]
