"""Tercer adaptador de `PerceptionPort` (shared_kernel): en vez de "ground
truth" de CoppeliaSim (todavía sin resolver del todo -- ver ROADMAP.md
Bloque 3, falta decidir cómo leer el radio de una esfera vía la API ZMQ) o
de recibir eventos programáticos como `PseudoPerceptionAdapter`, este lee
obstáculos de un fichero de texto plano, RELEYÉNDOLO ENTERO en cada
llamada a `get_scene()` -- sin diff, sin llevar la cuenta de "qué línea es
nueva".

Sirve como banco de pruebas mínimo de percepción sin depender de
CoppeliaSim: editar el fichero a mano desde otra terminal (añadir/mover/
quitar una línea) y que el siguiente `get_scene()` lo refleje es la forma
más simple posible de probar "el mundo cambió y el sistema se entera",
sin cámara, sin simulador. El adaptador en sí sigue sin saber nada de
ROS2 -- eso lo pone `perception_node/node.py` por encima, publicando
periódicamente lo que este `get_scene()` devuelva en `/perception/scene`
(`Commander` lo escucha desde ahí, ver docs/nodos_ros2.md §4).

Mismo papel que `StaticPerceptionAdapter` tuvo para dejar el puerto en pie
sin resolver percepción de verdad -- la diferencia es que aquí la fuente
SÍ cambia con el tiempo (el fichero), solo que el mecanismo para
enterarse es "pregúntale nuevo" (poll), no un evento.

Formato del fichero, una línea por entrada, de dos formas posibles:
    nombre x y z radio     -- un obstáculo (Scene.obstacles)
    objetivo x y z         -- EL objetivo a alcanzar (Scene.objects["objetivo"])

La palabra clave "objetivo" es reservada -- distingue la línea de goal de
un obstáculo por el número de campos (4 contra 5) y por empezar
literalmente por esa palabra; no nombres un obstáculo "objetivo" o esta
lectura lo interpretará como el goal. Solo puede haber UN objetivo activo
a la vez (clave fija "objetivo" en `Scene.objects`, no un nombre libre por
línea como los obstáculos) -- pensado para el caso de uso real: "muévete
aquí", no una lista de candidatos.

Líneas vacías y las que empiezan por "#" se ignoran (comentarios). El
nombre de un obstáculo es su clave en `Scene.obstacles` (Dict[str,
SphereObstacle]) -- repetirlo en el fichero se queda con la ÚLTIMA línea
que lo usa, mismo criterio "sobrescribe por clave" que `Scene.with_obstacle`.
"""

from __future__ import annotations

from pathlib import Path

from shared_kernel import Point, Scene, SphereObstacle

_COMMENT_PREFIX = "#"
_GOAL_KEYWORD = "objetivo"


class FilePerceptionAdapter:
    description = {
        "name": "file_perceptor",
        "description": (
            "Percepción vía fichero de texto plano, releído entero en "
            "cada consulta -- banco de pruebas para la tubería perceptor "
            "-> ensamblado -> planificación, sin cámara ni simulador."
        ),
        "reports": {
            "obstacles": (
                "Una línea por obstáculo: 'nombre x y z radio', en el "
                "marco cartesiano relativo a base_link (mismo que exige "
                "KinematicsPort). El nombre es la clave en "
                "Scene.obstacles -- repetirlo sobrescribe."
            ),
            "objects": (
                "Una línea 'objetivo x y z' -- el único objeto reportado, "
                "siempre bajo la clave fija 'objetivo' en Scene.objects, "
                "mismo marco que los obstáculos."
            ),
        },
    }

    def __init__(self, path: str) -> None:
        self._path = Path(path)

    def get_scene(self) -> Scene:
        scene = Scene.empty()
        for line in self._path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith(_COMMENT_PREFIX):
                continue
            fields = line.split()
            if fields[0] == _GOAL_KEYWORD:
                _, x, y, z = fields
                scene = scene.with_object(
                    _GOAL_KEYWORD, Point(float(x), float(y), float(z))
                )
                continue
            name, x, y, z, radius = fields
            obstacle = SphereObstacle(
                center=Point(float(x), float(y), float(z)), radius=float(radius)
            )
            scene = scene.with_obstacle(name, obstacle)
        return scene
