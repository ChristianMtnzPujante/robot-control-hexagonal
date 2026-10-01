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
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shared_kernel import Body, Box, Cylinder, JointConfiguration, Scene, Sphere

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

# assets/ vive en la raíz del repo (ver assets/robotiq_2f_85/README.md).
# resolve() hace falta con `colcon build --symlink-install`: sin él,
# __file__ apunta a build/ y parents[3] no sería la raíz.
_ASSETS_DIR = Path(__file__).resolve().parents[3] / "assets"

# Todos los cuerpos de `Scene.bodies` cuelgan de este dummy, para poder
# borrarlos de una vez al reconstruir (ver `_clear_previous_build`).
_BODIES_ROOT_ALIAS = "cuerpos_escena"
_GRASPABLE_COLOR = [0.85, 0.2, 0.15]  # rojo: se puede coger
_FIXED_COLOR = [0.6, 0.6, 0.6]  # gris: fijo (mesa, pared)


@dataclass(frozen=True)
class ToolMount:
    """Una herramienta (pinza, más adelante una cámara...) importada desde
    su PROPIO URDF y colgada de un joint del robot -- en vez de un URDF
    combinado robot+herramienta: cada pieza es un puerto distinto
    (RobotConnectorPort, GripperPort...) y puede no estar, y el URDF del
    robot es el que leen PoE/GA, verificado al micrómetro contra esta misma
    escena. Ver "Decisiones de Diseño Clave" en el vault (30/09).

    `parent_joint` es el joint del robot del que cuelga (joint6 = brida del
    CR5); `offset_pose` es la pose [x y z qx qy qz qw] de la base de la
    herramienta respecto al frame de ese joint con el robot a cero -- por
    defecto, justo en la brida sin girar. `root_link_visual_alias` sirve
    para localizarla, igual que el de `build_scene`."""

    urdf_path: str
    urdf_package_prefix: str
    parent_joint: str
    root_link_visual_alias: str
    offset_pose: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


ROBOTIQ_2F_85_URDF_PATH = str(_ASSETS_DIR / "robotiq_2f_85/urdf/robotiq_2f_85.urdf")
ROBOTIQ_2F_85_DRIVEN_JOINT = "robotiq_85_left_knuckle_joint"
# Montaje directo en la brida del CR5, sin acoplador intermedio: el eje z de
# Link6 sale de la brida y los dedos de la 2F-85 apuntan a +z desde su base.
# Si el acoplador real añade altura, va en offset_pose.
ROBOTIQ_2F_85_ON_CR5 = ToolMount(
    urdf_path=ROBOTIQ_2F_85_URDF_PATH,
    urdf_package_prefix=str(_ASSETS_DIR) + "/",
    parent_joint="joint6",
    root_link_visual_alias="robotiq_85_base_link_visual",
)


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
    mounts: Sequence[ToolMount] = (),
) -> "CoppeliaSimRobotAdapter":
    """Envoltorio de `build_scene` con los datos concretos del CR5 --
    conservado por compatibilidad con los demos que ya lo llaman así
    (`avoid_obstacle_demo.py`, etc.). Ver `build_scene` para el mecanismo
    real, ya generalizado a cualquier URDF (ver ROADMAP.md, Bloque 9,
    verificado en vivo importando también un Franka Panda de 7 GDL junto
    al CR5)."""
    return build_scene(
        port=port,
        urdf_path=_CR5_URDF_PATH,
        urdf_package_prefix=_CR5_URDF_PACKAGE_PREFIX,
        joint_names=_CR5_JOINT_NAMES,
        tip_name=_CR5_TIP_NAME,
        root_link_visual_alias="dummy_link_visual",
        initial_configuration=initial_configuration,
        scene=scene,
        mounts=mounts,
    )


def build_scene(
    port: int,
    urdf_path: str,
    urdf_package_prefix: str,
    joint_names: List[str],
    tip_name: str,
    root_link_visual_alias: str,
    initial_configuration: JointConfiguration,
    scene: Scene,
    mounts: Sequence[ToolMount] = (),
) -> "CoppeliaSimRobotAdapter":
    """Construye desde cero, en la escena actualmente cargada en el puerto
    `port`: el robot importado de `urdf_path`, en la postura
    `initial_configuration`, más un marcador visual por cada
    `SphereObstacle` de `scene.obstacles` y un shape por cada cuerpo de
    `scene.bodies` (ver `_render_bodies`). Sin ningún marcador de goal: el
    goal es un objetivo de `KinematicsPort`/`PlanningPort`, no algo que
    `Scene` conozca (ver `geometry_kernel/scene.py`) -- márcalo aparte con
    `.mark_goal(...)` sobre el `CoppeliaSimRobotAdapter` que devuelve esta
    función.

    `root_link_visual_alias` es el alias que `simURDF.importFile` le da al
    objeto de nivel superior del modelo importado -- por convención,
    `"<root_link>_visual"` (ver `_clear_previous_build`), necesario para
    poder reconstruir la escena sin acumular robots duplicados si se
    reutiliza la misma instancia de CoppeliaSim.

    Deja la simulación en marcha al terminar y devuelve un
    `CoppeliaSimRobotAdapter` ya conectado a `joint_names`/`tip_name`
    recién importados -- listo para usar como `RobotConnectorPort` sin
    volver a resolver handles."""
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
    _clear_previous_build(sim, root_link_visual_alias)

    _, model_handles = simURDF.importFile(
        urdf_path, _IMPORT_OPTIONS, urdf_package_prefix
    )
    # simURDF.import deja el modelo en modo DINÁMICO (física real) --
    # descubierto en vivo, en dos capas: (1) los joints en
    # jointmode_dynamic, por lo que `set_joints` (que solo llama
    # sim.setJointPosition, pensado para joints cinemáticos, ver
    # RobotConnectorPort) no se sostiene entre waypoints; (2) aunque se
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
    for name in joint_names:
        handle = sim.getObject(f"/{name}")
        sim.setJointMode(handle, sim.jointmode_kinematic)
    # Las herramientas se montan ANTES de poner la postura inicial: con el
    # robot a cero, el frame de cada joint es el de su link hijo.
    for mount in mounts:
        _mount_tool(sim, simURDF, mount)
    _render_bodies(sim, scene)
    for position in initial_configuration.positions:
        handle = sim.getObject(f"/{position.joint_name}")
        sim.setJointPosition(handle, position.angle_radians)

    sim.startSimulation()

    robot = CoppeliaSimRobotAdapter(
        joint_names=joint_names, tip_name=tip_name, zmq_port=port
    )
    for obstacle in scene.obstacles.values():
        robot.mark_obstacle(obstacle)
    return robot


def _mount_tool(sim, simURDF, mount: ToolMount) -> None:
    """Importa `mount.urdf_path` en el origen del mundo y lo cuelga de
    `mount.parent_joint`, que debe estar todavía a cero. Hija del JOINT (no
    del shape del link) porque así cuelga simURDF los propios links: el
    hijo de un joint gira con él. Cinemática y no dinámica, igual que el
    robot (ver el comentario de `build_scene`); sin ello `setJointPosition`
    no sostiene los dedos.

    Los `<mimic>` NO se traducen a nada en CoppeliaSim: con los joints en
    cinemático, quien mueve la herramienta escribe todos (ver
    `CoppeliaSimGripperAdapter`)."""
    _, handles = simURDF.importFile(
        mount.urdf_path, _IMPORT_OPTIONS, mount.urdf_package_prefix
    )
    root = handles[0]
    sim.setModelProperty(
        root, sim.getModelProperty(root) | sim.modelproperty_not_dynamic
    )
    for handle in sim.getObjectsInTree(root, sim.object_joint_type, 0):
        sim.setJointMode(handle, sim.jointmode_kinematic)
    parent = sim.getObject(f"/{mount.parent_joint}")
    world_pose = sim.multiplyPoses(
        sim.getObjectPose(parent, -1), list(mount.offset_pose)
    )
    sim.setObjectPose(root, -1, world_pose)
    sim.setObjectParent(root, parent, True)


def _render_bodies(sim, scene: Scene) -> None:
    """Crea un shape primitivo por cada cuerpo de `scene.bodies`, con alias
    igual a su nombre, en su pose. Estáticos y no "respondable": el robot
    también es cinemático, y así nada se cae ni empuja a nada al arrancar la
    simulación. Coger un cuerpo, cuando llegue, será cambiarle el padre a
    la pinza (agarre cinemático), no física.

    Sin datos propios en el shape (tipo, medidas, `graspable`): de momento
    la fuente de verdad es la `Scene` que los creó. Si algún día un
    `PerceptionPort` tiene que leerlos de vuelta desde CoppeliaSim, habrá
    que guardarlos aquí."""
    if not scene.bodies:
        return
    root = sim.createDummy(0.01)
    sim.setObjectAlias(root, _BODIES_ROOT_ALIAS)
    for name, body in scene.bodies.items():
        handle = _create_body_shape(sim, body)
        sim.setObjectAlias(handle, name)
        color = list(body.color) if body.color else (
            _GRASPABLE_COLOR if body.graspable else _FIXED_COLOR
        )
        sim.setShapeColor(handle, "", sim.colorcomponent_ambient_diffuse, color)
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
        pose = body.pose
        sim.setObjectPose(
            handle, -1, [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]
        )
        sim.setObjectParent(handle, root, True)


def _create_body_shape(sim, body: Body) -> int:
    """Medidas en el formato de `createPrimitiveShape`: tamaño TOTAL en
    x, y, z (un cilindro es [diámetro, diámetro, altura], con el eje en z,
    igual que `Cylinder`)."""
    shape = body.shape
    if isinstance(shape, Box):
        return sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [shape.size_x, shape.size_y, shape.size_z]
        )
    if isinstance(shape, Cylinder):
        diameter = 2 * shape.radius
        return sim.createPrimitiveShape(
            sim.primitiveshape_cylinder, [diameter, diameter, shape.height]
        )
    if isinstance(shape, Sphere):
        return sim.createPrimitiveShape(
            sim.primitiveshape_spheroid, [2 * shape.radius] * 3
        )
    raise TypeError(f"forma no soportada: {type(shape).__name__}")


def _robotiq_2f_85_grasp():
    """Geometría de agarre de la 2F-85, calculada el 30/09 de sus mallas de
    colisión (`meshes/collision/*_finger_tip.stl`) y de la cadena del URDF,
    en el marco de `GraspGeometry` (origen entre los nudillos, que están a
    0.0549 m de la base): las yemas van de z = 0.0923 a 0.1631 m de la base,
    miden 2.7 cm en y, y su cara interior está a 42.5 mm del plano medio
    abierta (85 mm de carrera, como la ficha de Robotiq) y a 0 cerrada. La
    tabla es la semiapertura cada 0.1 rad del nudillo (0..0.8)."""
    from robot_node.adapters.coppeliasim_gripper_adapter import GraspGeometry

    knuckle_z = 0.0549
    return GraspGeometry(
        left_knuckle_joint="robotiq_85_left_knuckle_joint",
        right_knuckle_joint="robotiq_85_right_knuckle_joint",
        left_tip_joint="robotiq_85_left_finger_tip_joint",
        right_tip_joint="robotiq_85_right_finger_tip_joint",
        pad_z_range=(0.0923 - knuckle_z, 0.1631 - knuckle_z),
        pad_half_width=0.0135,
        half_gap_by_fraction=(
            0.0425, 0.0380, 0.0331, 0.0280, 0.0227, 0.0171, 0.0115, 0.0058, 0.0001,
        ),
    )


# Dónde queda, en el marco de la brida (joint6), el centro de lo que la
# 2F-85 agarra para que las yemas lo cubran sin tocar lo que tenga debajo:
# a 5 cm de ancho, la punta de las yemas llega a 0.159 m de la base, así
# que con el centro a 0.14 m sobra ~0.5 cm hasta la mesa bajo un cubo de 5 cm.
ROBOTIQ_2F_85_GRASP_DEPTH = 0.14


def robotiq_2f_85_gripper(
    port: int, scene: Optional[Scene] = None
) -> "CoppeliaSimGripperAdapter":
    """`GripperPort` para la 2F-85 de una escena construida con
    `mounts=[ROBOTIQ_2F_85_ON_CR5]`. Los joints y multiplicadores salen del
    mismo URDF que se importó. Con `scene`, además agarra (cinemático) los
    cuerpos `graspable` de esa escena."""
    from robot_node.adapters.coppeliasim_gripper_adapter import (
        CoppeliaSimGripperAdapter,
        gripper_joints_from_urdf,
    )

    sim = RemoteAPIClient(port=port).require("sim")
    return CoppeliaSimGripperAdapter(
        sim,
        gripper_joints_from_urdf(ROBOTIQ_2F_85_URDF_PATH, ROBOTIQ_2F_85_DRIVEN_JOINT),
        grasp=_robotiq_2f_85_grasp() if scene is not None else None,
        graspable_bodies=scene.graspable_bodies() if scene is not None else None,
    )


def _clear_previous_build(sim, root_link_visual_alias: str) -> None:
    """Idempotente: si ya se había construido una escena en esta misma
    instancia de CoppeliaSim (reutilizada entre dos ejecuciones del demo),
    borra el robot importado anteriormente (por `sim.removeModel`, ya que
    `simURDF.importFile` lo marca como modelo -- borra todo el árbol de una
    vez) y los marcadores de obstáculo/objetivo/trail de la ejecución
    anterior, antes de reimportar. Sin esto, cada reutilización de la misma
    instancia acumularía un robot duplicado (`joint1`, `joint1_2`, ...) con
    handles ambiguos.

    Solo mira objetos de primer nivel (sin padre): todo lo demás cuelga de
    alguno de esos como hijo y se borra con él."""
    handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    top_level = [
        (handle, sim.getObjectAlias(handle))
        for handle in handles
        if sim.getObjectParent(handle) == -1
    ]
    for handle, alias in top_level:
        if alias == root_link_visual_alias:
            sim.removeModel(handle)
        elif alias == _BODIES_ROOT_ALIAS:
            sim.removeObjects(list(sim.getObjectsInTree(handle, sim.handle_all, 0)))
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
