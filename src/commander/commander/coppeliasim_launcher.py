"""Arranca (o reutiliza) una instancia de CoppeliaSim para una sesión de
control, cargada con la escena que le corresponde y en play -- para que
robot_node/controller_node puedan arrancar después asumiendo que la escena
ya está lista (sin condición de carrera con simIK.addElementFromScene
buscando objetos que aún no existen).

Solo lo usa two_sessions_demo.py -- no es un puerto del dominio, es
orquestación de proceso específica de esta demo.

Modo headless (-h) de CoppeliaSim no es fiable en este entorno (el proceso
termina solo a los pocos segundos) -- se lanza siempre con GUI. Dos
instancias a la vez necesitan puerto ZMQ distinto
(-GzmqRemoteApi.rpcPort=<puerto>, confirmado que lo fuerza) y carpeta de
settings distinta (env var COPPELIASIM_USER_SETTINGS_FOLDER_SUFFIX) -- sin
esto último, dos instancias compiten por el mismo usrset.txt y ninguna
termina de arrancar.
"""

from __future__ import annotations

import os
import socket
import subprocess
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

_COPPELIASIM_SH = os.path.expanduser(
    "~/RoboticInvest/CoppeliaSim/coppeliaSim.sh"
)


class CoppeliaSimLaunchError(Exception):
    pass


def ensure_coppeliasim_scene(
    port: int,
    settings_suffix: str,
    scene_path: str,
    timeout: float = 90.0,
) -> None:
    """Deja una instancia de CoppeliaSim en `port` con `scene_path` cargada
    y en play. Si ya hay algo escuchando en `port`, se reutiliza tal cual
    (se asume que es una instancia de CoppeliaSim válida) y solo se le
    manda cargar la escena; si no, se lanza una instancia nueva.
    """
    if not _port_open(port):
        _launch(port, settings_suffix)
        if not _wait_for_port(port, timeout):
            raise CoppeliaSimLaunchError(
                f"CoppeliaSim no respondió en el puerto {port} tras "
                f"{timeout:.0f}s. Ábrelo tú mismo con:\n"
                f'  COPPELIASIM_USER_SETTINGS_FOLDER_SUFFIX="{settings_suffix}" '
                f'"{_COPPELIASIM_SH}" -GzmqRemoteApi.rpcPort={port}\n'
                f"y carga la escena {scene_path} en play."
            )

    client = RemoteAPIClient(port=port)
    sim = client.require("sim")
    # loadScene exige la simulación parada -- stopSimulation es idempotente
    # si ya lo estaba (mismo patrón que
    # CoppeliaSimRobotAdapter._load_scene_and_play).
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.1)
    sim.loadScene(scene_path)
    sim.startSimulation()


def _port_open(port: int, host: str = "localhost") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.5)
        return probe.connect_ex((host, port)) == 0


def _wait_for_port(port: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _port_open(port):
            return True
        time.sleep(1.0)
    return False


def _launch(port: int, settings_suffix: str) -> None:
    env = dict(os.environ)
    env["COPPELIASIM_USER_SETTINGS_FOLDER_SUFFIX"] = settings_suffix
    subprocess.Popen(
        [_COPPELIASIM_SH, f"-GzmqRemoteApi.rpcPort={port}"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
