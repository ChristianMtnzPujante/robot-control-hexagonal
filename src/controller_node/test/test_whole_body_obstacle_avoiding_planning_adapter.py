import numpy as np

from shared_kernel import (
    JointConfiguration,
    JointPosition,
    Point,
    Pose,
    Scene,
    SphereObstacle,
    Trajectory,
)

from controller_node.adapters.whole_body_obstacle_avoiding_planning_adapter import (
    WholeBodyObstacleAvoidingPlanningAdapter,
    _within_a_full_turn,
)

_CONFIGURATION = JointConfiguration.create(
    [
        JointPosition("elbow_x", 0.0),
        JointPosition("elbow_y", 0.0),
        JointPosition("tip_x", 0.0),
        JointPosition("tip_y", 0.0),
    ]
).value


class _TwoLinkStubKinematicsPort:
    """Cinemática de prueba: un brazo sintético de 2 eslabones (codo + tip)
    en el plano XY. `compute_trajectory` sitúa el tip exactamente en el
    goal pedido, y el codo a medio camino entre el tip actual y el nuevo
    tip MÁS un "bulto" perpendicular fijo (`bulge`) -- así un via-point que
    cambie la componente Y del objetivo mueve también al codo, pudiendo
    sacarlo de un obstáculo, igual que en un brazo real el codo se arrastra
    con la mano.
    """

    def __init__(self, bulge: float = 1.0) -> None:
        self._bulge = bulge
        self.received_goals: list = []

    def compute_trajectory(
        self, goal: Pose, current_configuration: JointConfiguration
    ) -> Trajectory:
        self.received_goals.append(goal)
        current_tip_x = current_configuration.angle_of("tip_x")
        current_tip_y = current_configuration.angle_of("tip_y")
        target = JointConfiguration.create(
            [
                JointPosition("elbow_x", (current_tip_x + goal.x) / 2),
                JointPosition(
                    "elbow_y", (current_tip_y + goal.y) / 2 + self._bulge
                ),
                JointPosition("tip_x", goal.x),
                JointPosition("tip_y", goal.y),
            ]
        ).value
        return Trajectory.create([current_configuration, target]).value

    def link_poses(self, configuration: JointConfiguration) -> list:
        return [
            Pose(
                x=configuration.angle_of("elbow_x"),
                y=configuration.angle_of("elbow_y"),
                z=0.0,
            ),
            Pose(
                x=configuration.angle_of("tip_x"),
                y=configuration.angle_of("tip_y"),
                z=0.0,
            ),
        ]


def test_no_body_collision_returns_direct_trajectory_unchanged():
    kinematics = _TwoLinkStubKinematicsPort(bulge=0.0)
    planner = WholeBodyObstacleAvoidingPlanningAdapter(kinematics, clearance=0.05)
    goal = Pose(x=2.0, y=0.0, z=0.0)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, Scene.empty())

    assert kinematics.received_goals == [goal]
    assert len(trajectory.waypoints) == 2


def test_detects_a_body_collision_that_a_tip_only_check_would_miss():
    # bulge=1.0: el tip va en linea recta por y=0 (nunca pasa cerca del
    # obstaculo), pero el codo se va a y=1 -- un chequeo que solo mirase el
    # segmento del tip (ObstacleAvoidingPlanningAdapter) no vería nada.
    kinematics = _TwoLinkStubKinematicsPort(bulge=1.0)
    # Necesita varios reintentos: empujar el via-point justo a
    # radius+clearance del centro no basta para que el SEGMENTO
    # codo->tip (que se acerca en ángulo) también quede libre -- el primer
    # empuje suele "cortar la esquina" del obstáculo. Con
    # detour_growth_factor=1.5 por defecto, converge en torno al 7º intento.
    planner = WholeBodyObstacleAvoidingPlanningAdapter(
        kinematics, clearance=0.05, max_detour_attempts=10
    )
    goal = Pose(x=2.0, y=0.0, z=0.0)
    obstacle = SphereObstacle(center=Point(1.0, 1.0, 0.0), radius=0.1)
    scene = Scene.empty().with_obstacle(obstacle)

    # El chequeo del tip-only (segmento (0,0,0)->(2,0,0)) no detecta nada:
    # el obstaculo esta a distancia 1.0 de esa recta, muy por encima de
    # radius+clearance=0.15.
    tip_only_distance = 1.0
    assert tip_only_distance > obstacle.radius + 0.05

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    # El planificador SÍ lo detectó: intentó al menos un desvío (más de una
    # llamada a compute_trajectory con goals distintos del original).
    assert len(kinematics.received_goals) > 1
    # Y consiguió dejar el cuerpo entero libre en la trayectoria final.
    assert planner._worst_body_hit(trajectory, scene.obstacles) is None


def test_gives_up_gracefully_when_no_detour_can_clear_the_body():
    # Un obstaculo gigante, centrado justo donde el chequeo del tip-only
    # buscaría el punto de desvío (el punto medio del segmento base->goal
    # del tip), hace que _worst_body_hit siga encontrando colisión por
    # mucho margen que se pruebe -- el planificador no debe lanzar excepción,
    # solo devolver su mejor intento.
    kinematics = _TwoLinkStubKinematicsPort(bulge=1.0)
    planner = WholeBodyObstacleAvoidingPlanningAdapter(
        kinematics, clearance=0.05, max_detour_attempts=3
    )
    goal = Pose(x=2.0, y=0.0, z=0.0)
    obstacle = SphereObstacle(center=Point(1.0, 0.0, 0.0), radius=50.0)
    scene = Scene.empty().with_obstacle(obstacle)

    trajectory = planner.compute_trajectory(goal, _CONFIGURATION, scene)

    assert trajectory.waypoints
    # Se agotaron los intentos: 1 llamada directa + 2 por cada intento de
    # desvío (to_via + to_goal).
    assert len(kinematics.received_goals) == 1 + 3 * 2


def test_within_a_full_turn_rejects_angles_beyond_plus_minus_2pi():
    # Regresión de un hallazgo real: Newton-Raphson puede converger a un
    # ángulo matemáticamente válido (misma pose, módulo 2π) pero muy fuera
    # de rango -- p. ej. 540° en vez de 180° -- que un joint real (o el
    # límite ±2π ya configurado en CoppeliaSim al importar el CR5, ver
    # coppeliasim_scene_builder.py) recorta en silencio.
    within_range = JointConfiguration.create(
        [JointPosition("joint1", 3.0), JointPosition("joint2", -3.0)]
    ).value
    beyond_range = JointConfiguration.create(
        [JointPosition("joint1", 0.0), JointPosition("joint2", 9.42)]  # 540°
    ).value

    assert _within_a_full_turn(Trajectory.create([within_range]).value) is True
    assert _within_a_full_turn(Trajectory.create([beyond_range]).value) is False
    assert (
        _within_a_full_turn(Trajectory.create([within_range, beyond_range]).value)
        is False
    )
