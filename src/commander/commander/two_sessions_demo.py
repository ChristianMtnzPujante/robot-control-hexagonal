"""Demo: dos ControlSession simultáneas e independientes, cada una con su
propia instancia de CoppeliaSim y su propio cálculo de cinemática --
ejemplo de que cambiar de robot o de estrategia de cálculo es cuestión de
parámetros de create_session, no de tocar código (RobotControllerPort /
KinematicsPort son los únicos puntos de contacto).

Sesión A: escena cr5_base.ttt, cinemática PoeKinematicsAdapter (matemática
propia, sin depender de CoppeliaSim para resolver IK).
Sesión B: escena cr5_base_pruebas.ttt, cinemática
CoppeliaSimIkKinematicsAdapter (delega en simIK del simulador).

Abre DOS ventanas de CoppeliaSim (el modo headless no es fiable en este
entorno -- ver coppeliasim_launcher.py) si no hay ya instancias corriendo
en los puertos 23000/23001; si ya las hay, las reutiliza.

Los dos Pose objetivo NO son intercambiables entre sesiones, y es
deliberado, no un descuido: PoeKinematicsAdapter resuelve la pose relativa
a base_link (así están derivados sus twists), mientras que
CoppeliaSimIkKinematicsAdapter pasa goal.x/y/z tal cual como coordenadas de
MUNDO a simIK (ver su propio código). KinematicsPort no especifica en qué
frame va Pose -- hallazgo real al comparar ambos adaptadores, no arreglado
aquí (queda para cuando se decida un frame único para todo KinematicsPort).
Ambos valores usados abajo están verificados numéricamente: cada uno
converge en su sesión con error sub-milimétrico desde el arranque real de
la escena.
"""

from __future__ import annotations

import os

import rclpy
from ros2_kit import shutdown_node

from shared_kernel import Pose

from .commander_node import Commander
from .coppeliasim_launcher import ensure_coppeliasim_scene

_SCENES_DIR = os.path.expanduser("~/RoboticInvest/CoppeliaSim/scenes")
_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

_SESSION_A = {
    "name": "poe",
    "port": 23000,
    "settings_suffix": "_session_poe",
    "scene_path": os.path.join(_SCENES_DIR, "cr5_base.ttt"),
    "controller_strategy": "poe",
    # Relativo a base_link -- ver docstring del módulo.
    "goal": Pose(
        x=0.37822, y=-0.23471, z=0.458,
        qx=0.70686, qy=0.00707, qz=0.03534, qw=0.70644,
    ),
}

_SESSION_B = {
    "name": "coppelia_ik",
    "port": 23001,
    "settings_suffix": "_session_coppelia_ik",
    "scene_path": os.path.join(_SCENES_DIR, "cr5_base_pruebas.ttt"),
    "controller_strategy": "coppeliasim_ik",
    # Coordenadas de mundo -- mismo valor que ya usa commander_demo (mismo
    # estado inicial de escena) -- ver docstring del módulo.
    "goal": Pose(x=-0.139, y=-0.465, z=0.096),
}


def _start_session(commander: Commander, config: dict) -> None:
    ensure_coppeliasim_scene(
        port=config["port"],
        settings_suffix=config["settings_suffix"],
        scene_path=config["scene_path"],
    )
    commander.create_session(
        name=config["name"],
        robot_target="simulado",
        controller_strategy=config["controller_strategy"],
        joint_names=_JOINT_NAMES,
        waypoint_period_seconds=1.5,
        tip_name="Link6_visual",
        zmq_port=config["port"],
        # scene_path="" (default): la escena ya la cargó
        # ensure_coppeliasim_scene arriba, antes de arrancar robot_node/
        # controller_node -- evita que CoppeliaSimIkKinematicsAdapter busque
        # base_link_respondable/Link6_visual antes de que existan.
    )
    commander.send_goal(config["name"], config["goal"])


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()

    _start_session(commander, _SESSION_A)
    _start_session(commander, _SESSION_B)

    try:
        rclpy.spin(commander)
    finally:
        commander.close_session(_SESSION_A["name"])
        commander.close_session(_SESSION_B["name"])
        shutdown_node(commander)


if __name__ == "__main__":
    main()
