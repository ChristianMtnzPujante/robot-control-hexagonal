from shared_kernel import Point, Scene, SphereObstacle

from perception_node.adapters.pseudo_perception_adapter import PseudoPerceptionAdapter


def test_starts_empty_without_an_initial_scene():
    adapter = PseudoPerceptionAdapter()

    assert adapter.get_scene() == Scene.empty()


def test_starts_from_a_given_initial_scene():
    scene = Scene.empty().with_object("caja", Point(0.1, 0.2, 0.3))

    adapter = PseudoPerceptionAdapter(scene)

    assert adapter.get_scene() == scene


def test_report_obstacle_accumulates_into_the_scene():
    adapter = PseudoPerceptionAdapter()
    obstacle = SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05)

    adapter.report_obstacle("caja", obstacle)

    assert adapter.get_scene().obstacles == {"caja": obstacle}


def test_report_object_accumulates_into_the_scene():
    adapter = PseudoPerceptionAdapter()

    adapter.report_object("objetivo", Point(0.4, 0.1, 0.5))

    assert adapter.get_scene().objects == {"objetivo": Point(0.4, 0.1, 0.5)}


def test_multiple_reports_accumulate_without_losing_previous_ones():
    adapter = PseudoPerceptionAdapter()
    first = SphereObstacle(center=Point(0.1, 0.0, 0.0), radius=0.05)
    second = SphereObstacle(center=Point(0.2, 0.0, 0.0), radius=0.05)

    adapter.report_obstacle("uno", first)
    adapter.report_obstacle("dos", second)
    adapter.report_object("objetivo", Point(0.0, 0.0, 1.0))

    scene = adapter.get_scene()
    assert scene.obstacles == {"uno": first, "dos": second}
    assert scene.objects == {"objetivo": Point(0.0, 0.0, 1.0)}


def test_get_scene_reflects_the_latest_state_each_call():
    adapter = PseudoPerceptionAdapter()

    before = adapter.get_scene()
    adapter.report_obstacle("caja", SphereObstacle(center=Point(0.0, 0.0, 0.0), radius=0.05))
    after = adapter.get_scene()

    # Scene es inmutable -- el snapshot leído ANTES de reportar no cambia
    # retroactivamente cuando llega un evento nuevo.
    assert before == Scene.empty()
    assert after != before
