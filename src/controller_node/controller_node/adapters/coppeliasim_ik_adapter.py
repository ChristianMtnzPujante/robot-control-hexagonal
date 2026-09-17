"""IK real, pero delegada en el solver `simIK` del propio CoppeliaSim --
no en matemática propia derivada a mano (eso sigue siendo lo pendiente de
verdad de PoE/GA/DH, ver esos adaptadores). Por eso este SÍ depende de
CoppeliaSim (rompe la promesa de `KinematicsPort` de ser agnóstico de
plataforma): es una decisión consciente para tener una IK físicamente
correcta ya, a cambio de acoplarse al simulador.

Solo restringe posición (`simIK.constraint_position`) -- la orientación
del `Pose` objetivo se ignora, igual que en el resto de la demo.

Resuelve sobre un ENTORNO IK AISLADO (`simIK.createEnvironment`), clonando
la cadena cinemática real pero sin tocar la escena real al resolver --
así "calcular la trayectoria" no mueve el robot visible antes de que
empiece a enviarse de verdad por el pipeline de siempre (topics ->
robot_node). El objetivo cartesiano se mueve siempre en el CLON, nunca en
la escena real.

`simIK` resuelve por iteración local desde la configuración actual: para
saltos grandes puede no converger (límites de articulación, pasos
demasiado grandes). Si eso pasa, se lanza un error explícito en vez de
devolver una trayectoria hacia un sitio equivocado.

`forward_kinematics`/`link_poses` (parte formal de `KinematicsPort` desde
el 08/09, ver `shared_kernel/ports.py`) se resuelven sobre el MISMO
entorno IK aislado que `compute_trajectory` -- se fija cada joint clon a
los ángulos de la `JointConfiguration` recibida (`simIK.setJointPosition`,
sin invocar el solver) y se lee de vuelta la pose del objeto clon que
corresponda (`simIK.getObjectPose`) -- igual que `compute_trajectory` NO
toca la escena real al calcular, solo el clon. A diferencia de PoE, aquí
NO se garantiza `link_poses(...)[-1] == forward_kinematics(...)`: `tip`
puede tener un offset propio respecto a la última articulación (p. ej. una
malla visual montada más allá del último joint) -- cada uno se lee de su
propio objeto real, sin asumir que coinciden.

NOTA: implementado sin poder verificarlo en vivo contra una instancia real
de CoppeliaSim en este entorno (requiere GUI, no disponible aquí) -- antes
de confiar en esto en una sesión real, comprobarlo contra la escena
`cr5_base.ttt` igual que se hizo con el resto del adaptador.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shared_kernel import JointConfiguration, JointPosition, Pose, Trajectory


class CoppeliaSimIkKinematicsAdapter:
    def __init__(
        self,
        # Nombres de objetos de la escena cr5_base.ttt puestos como default
        # -- controller_node construye este adaptador sin argumentos
        # (node.py::_build_adapter), así que hoy no hay forma de cambiarlos
        # por sesión sin editar este archivo. Ver ROADMAP.md, Bloque 9.
        base_name: str = "base_link_respondable",
        tip_name: str = "Link6_visual",
        steps: int = 20,
        zmq_port: int = 23000,
    ):
        self._steps = steps
        self._client = RemoteAPIClient(port=zmq_port)
        self._sim = self._client.require("sim")
        self._simIK = self._client.require("simIK")

        base = self._sim.getObject(f"/{base_name}")
        tip = self._sim.getObject(f"/{tip_name}")
        self._tip = tip
        template_target = self._get_or_create_template_target(tip)

        self._env = self._simIK.createEnvironment()
        group = self._simIK.createGroup(self._env)
        _, real_to_clone, _ = self._simIK.addElementFromScene(
            self._env,
            group,
            base,
            tip,
            template_target,
            self._simIK.constraint_position,
        )
        self._group = group
        self._real_to_clone: Dict[int, int] = real_to_clone
        self._clone_target = real_to_clone[template_target]
        self._joint_clone_handles: Optional[Dict[str, int]] = None

    def _get_or_create_template_target(self, tip: int) -> int:
        # Solo sirve de plantilla para que addElementFromScene clone el
        # objetivo dentro del entorno IK -- después de esto no se vuelve a
        # tocar (el objetivo real de cada compute_trajectory se mueve
        # siempre en el clon). Se reutiliza entre lanzamientos para no
        # acumular dummies sueltos en la escena real.
        try:
            return self._sim.getObject("/ik_target_template")
        except Exception:
            handle = self._sim.createDummy(0.01)
            self._sim.setObjectAlias(handle, "ik_target_template")
            self._sim.setObjectPosition(handle, -1, self._sim.getObjectPosition(tip, -1))
            return handle

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        self._ensure_joint_mapping(current_configuration)

        self._simIK.setObjectPose(
            self._env, self._clone_target, -1, [goal.x, goal.y, goal.z, 0.0, 0.0, 0.0, 1.0]
        )
        result_code, _flags, _precision = self._simIK.handleGroup(
            self._env, self._group, {"syncWorlds": False}
        )
        if result_code != self._simIK.result_success:
            raise RuntimeError(
                f"CoppeliaSimIkKinematicsAdapter: simIK no convergió para "
                f"Pose(x={goal.x}, y={goal.y}, z={goal.z}) (código {result_code}). "
                "Prueba un objetivo más cercano a la posición actual del brazo."
            )

        target_positions = [
            JointPosition(
                name, self._simIK.getJointPosition(self._env, handle)
            )
            for name, handle in self._joint_clone_handles.items()
        ]
        target_configuration = JointConfiguration.create(target_positions).value
        return Trajectory.straight_line(
            current_configuration, target_configuration, self._steps
        )

    def _ensure_joint_mapping(self, current_configuration: JointConfiguration) -> None:
        if self._joint_clone_handles is not None:
            return
        mapping = {}
        for position in current_configuration.positions:
            real_handle = self._sim.getObject(f"/{position.joint_name}")
            mapping[position.joint_name] = self._real_to_clone[real_handle]
        self._joint_clone_handles = mapping

    def forward_kinematics(self, configuration: JointConfiguration) -> Pose:
        self._apply_configuration_to_clone(configuration)
        return self._read_clone_pose(self._real_to_clone[self._tip])

    def link_poses(self, configuration: JointConfiguration) -> List[Pose]:
        self._apply_configuration_to_clone(configuration)
        return [
            self._read_clone_pose(self._joint_clone_handles[position.joint_name])
            for position in configuration.positions
        ]

    def _apply_configuration_to_clone(self, configuration: JointConfiguration) -> None:
        # Fija cada joint del CLON a los ángulos recibidos, sin invocar
        # handleGroup -- no es una IK a resolver, es leer hacia atrás una
        # configuración ya conocida. No toca la escena real (mismo
        # aislamiento que compute_trajectory).
        self._ensure_joint_mapping(configuration)
        for position in configuration.positions:
            handle = self._joint_clone_handles[position.joint_name]
            self._simIK.setJointPosition(self._env, handle, position.angle_radians)

    def _read_clone_pose(self, handle: int) -> Pose:
        x, y, z, qx, qy, qz, qw = self._simIK.getObjectPose(self._env, handle, -1)
        return Pose(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)
