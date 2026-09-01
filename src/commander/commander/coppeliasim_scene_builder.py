"""Construye una escena de CoppeliaSim por código, a partir de una
descripción (qué robot, en qué postura inicial, qué `Scene` de dominio),
en vez de depender de un `.ttt` hecho a mano como `cr5_base.ttt` -- ver
ROADMAP.md Bloque 9 y el hallazgo de `two_sessions_demo.py` sobre el marco
interno de `KinematicsPort` frente al marco mundo de CoppeliaSim.

Importa el CR5 directamente desde su URDF real (el mismo archivo que ya usa
`urdf_kit.parse_urdf_file` para derivar el `RobotDescription` de
`PoeKinematicsAdapter`), usando el plugin `simURDF` de CoppeliaSim vía
`client.require("simURDF")` -- confirmado accesible por el mismo mecanismo
que `client.require("sim")`. Verificado empíricamente contra CoppeliaSim
real, importando SIN la opción "centrar modelo" (bit 32 de `_IMPORT_OPTIONS`,
ver `simURDF.import` en `lua/simURDF.lua`): `base_link_respondable` queda
exactamente en el origen del mundo con orientación identidad, y
`Link6_visual` en la configuración cero coincide con
`PoeKinematicsAdapter.forward_kinematics` al micrómetro. Esto ELIMINA el
hallazgo de marco sin resolver que documentaba `two_sessions_demo.py` y que
obligó a calibrar una transformación en la primera versión de
`avoid_obstacle_demo.py`: marco interno == marco mundo por construcción, ya
no hace falta ninguna calibración.

Deliberadamente NO vive en `geometry_kernel`/`Scene`: `Scene` es dominio
puro (sin depender de nada, ni siquiera de `shared_kernel` -- ver su propio
docstring) y debe seguir sin saber qué es CoppeliaSim. Este módulo hace lo
inverso de `perception_node/adapters/static_perception_adapter.py`: en vez
de PRODUCIR una `Scene` a partir del mundo, CONSUME una `Scene` (más una
`RobotDescription`/postura inicial) para construir un mundo -- misma
frontera hexagonal, dirección opuesta.
"""

from __future__ import annotations

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shared_kernel import JointConfiguration, Scene

from .coppeliasim_launcher import CoppeliaSimLaunchError, _launch, _port_open, _wait_for_port

_CR5_URDF_PATH = "/home/chris/ros2_ws/src/TCP-IP-ROS-6AXis/dobot_description/urdf/cr5_robot.urdf"
# simURDF.import sustituye el literal "package://" por este prefijo -- las
# mallas del URDF referencian "package://dobot_description/meshes/...", así
# que el prefijo debe ser el directorio que CONTIENE a dobot_description/
# (no dobot_description/ en sí, o el path quedaría duplicado).
_CR5_URDF_PACKAGE_PREFIX = "/home/chris/ros2_ws/src/TCP-IP-ROS-6AXis/"
_CR5_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_CR5_TIP_NAME = "Link6_visual"

# Bit-flags de simURDF.import (ver addOns/URDF importer.lua, misma
# combinación que trae por defecto el diálogo del importador salvo por el
# bit 32, que aquí se activa a propósito para NO recentrar el modelo -- lo
# queremos exactamente en el marco del URDF, sin "ayuda" de CoppeliaSim):
#   8   = crear visual si el link no trae uno
#   32  = NO centrar el modelo sobre el suelo (mantener el origen del URDF)
#   128 = alternateLocalRespondableMasks (default del importador)
_IMPORT_OPTIONS = 8 + 32 + 128


def ensure_coppeliasim_running(
    port: int, settings_suffix: str, timeout: float = 90.0
) -> None:
    """Deja una instancia de CoppeliaSim escuchando en `port`, SIN tocar
    qué escena tiene cargada ni su estado de simulación -- a diferencia de
    `coppeliasim_launcher.ensure_coppeliasim_scene`, que asume que quieres
    cargar un `.ttt` concreto. Aquí partimos de la escena en blanco con la
    que arranca CoppeliaSim (o de lo que ya hubiera en el puerto, si se
    reutiliza una instancia) y construimos todo por código a partir de ahí."""
    if _port_open(port):
        return
    _launch(port, settings_suffix)
    if not _wait_for_port(port, timeout):
        raise CoppeliaSimLaunchError(
            f"CoppeliaSim no respondió en el puerto {port} tras {timeout:.0f}s."
        )


def build_cr5_scene(
    port: int,
    initial_configuration: JointConfiguration,
    scene: Scene,
) -> "CoppeliaSimRobotAdapter":
    """Construye desde cero, en la escena actualmente cargada en el puerto
    `port`: el CR5 importado de su URDF real, en la postura
    `initial_configuration`, más un marcador visual por cada
    `SphereObstacle` de `scene.obstacles`. Sin ningún marcador de goal: el
    goal es un objetivo de `KinematicsPort`/`PlanningPort`, no algo que
    `Scene` conozca (ver `geometry_kernel/scene.py`) -- márcalo aparte con
    `.mark_goal(...)` sobre el `CoppeliaSimRobotAdapter` que devuelve esta
    función.

    Deja la simulación en marcha al terminar y devuelve un
    `CoppeliaSimRobotAdapter` ya conectado a los `joint1..joint6`/
    `Link6_visual` recién importados -- listo para usar como
    `RobotControllerPort` sin volver a resolver handles."""
    # Import perezoso: evita una dependencia circular con robot_node en el
    # nivel de módulo (commander ya depende de robot_node, ver package.xml,
    # así que esto es solo por orden de import, no una dependencia nueva).
    from robot_node.adapters.coppeliasim_adapter import CoppeliaSimRobotAdapter

    client = RemoteAPIClient(port=port)
    sim = client.require("sim")
    simURDF = client.require("simURDF")

    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.1)
    _clear_previous_build(sim)

    _, model_handles = simURDF.importFile(
        _CR5_URDF_PATH, _IMPORT_OPTIONS, _CR5_URDF_PACKAGE_PREFIX
    )
    # simURDF.import deja el modelo en modo DINÁMICO (física real) --
    # descubierto en vivo, en dos capas: (1) los joints en
    # jointmode_dynamic, por lo que `set_joints` (que solo llama
    # sim.setJointPosition, pensado para joints cinemáticos, ver
    # RobotControllerPort) no se sostiene entre waypoints; (2) aunque se
    # fuerce el joint a cinemático, los shapes "respondable" siguen
    # marcados dinámicos y el motor de físicas sigue moviéndolos de forma
    # independiente del árbol cinemático, produciendo posiciones finales
    # sin relación con la trayectoria calculada (el propio joint sí queda
    # con el ángulo correcto -- es la propagación a los shapes la que se
    # rompe). `cr5_base.ttt` ya traía todo esto en cinemático/estático a
    # mano; aquí hay que forzarlo tras importar, a los dos niveles.
    sim.setModelProperty(
        model_handles[0],
        sim.getModelProperty(model_handles[0]) | sim.modelproperty_not_dynamic,
    )
    for name in _CR5_JOINT_NAMES:
        handle = sim.getObject(f"/{name}")
        sim.setJointMode(handle, sim.jointmode_kinematic)
    for position in initial_configuration.positions:
        handle = sim.getObject(f"/{position.joint_name}")
        sim.setJointPosition(handle, position.angle_radians)

    sim.startSimulation()

    robot = CoppeliaSimRobotAdapter(
        joint_names=_CR5_JOINT_NAMES, tip_name=_CR5_TIP_NAME, zmq_port=port
    )
    for obstacle in scene.obstacles:
        robot.mark_obstacle(obstacle)
    return robot


def _clear_previous_build(sim) -> None:
    """`build_cr5_scene` es idempotente: si ya se había construido una
    escena en esta misma instancia de CoppeliaSim (reutilizada entre dos
    ejecuciones del demo), borra el CR5 importado anteriormente (por
    `sim.removeModel`, ya que `simURDF.importFile` lo marca como modelo --
    borra todo el árbol de una vez) y los marcadores de obstáculo/objetivo/
    trail de la ejecución anterior, antes de reimportar. Sin esto, cada
    reutilización de la misma instancia acumularía un CR5 duplicado
    (`joint1`, `joint1_2`, ...) con handles ambiguos.

    Solo mira objetos de primer nivel (sin padre): todo lo demás cuelga de
    alguno de esos como hijo y se borra con él."""
    handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    top_level = [
        (handle, sim.getObjectAlias(handle))
        for handle in handles
        if sim.getObjectParent(handle) == -1
    ]
    for handle, alias in top_level:
        if alias == "dummy_link_visual":
            sim.removeModel(handle)
        elif alias in ("objetivo", "obstaculo") or alias.startswith("waypoint_"):
            sim.removeObjects([handle])


def save_scene(port: int, path: str) -> None:
    """Persiste la escena actual (robot + marcadores ya colocados) como
    `.ttt` -- la descripción usada para construirla (ver `build_cr5_scene`)
    queda así 'guardada en la escena': se puede recargar directamente con
    `coppeliasim_launcher.ensure_coppeliasim_scene(scene_path=path)` sin
    reimportar el URDF cada vez, aunque el punto de partida sigue siendo
    reconstruirla desde la descripción, no depender de este archivo."""
    sim = RemoteAPIClient(port=port).require("sim")
    sim.saveScene(path)
