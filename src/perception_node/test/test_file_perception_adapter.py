from shared_kernel import Point, Scene, SphereObstacle

from perception_node.adapters.file_perception_adapter import FilePerceptionAdapter


def _write(path, text):
    path.write_text(text)
    return str(path)


def test_empty_file_yields_an_empty_scene(tmp_path):
    path = _write(tmp_path / "obstacles.txt", "")

    adapter = FilePerceptionAdapter(path)

    assert adapter.get_scene() == Scene.empty()


def test_parses_one_obstacle_per_line(tmp_path):
    path = _write(
        tmp_path / "obstacles.txt",
        "caja 0.3 0.0 0.2 0.05\n"
        "pared 0.1 0.4 0.5 0.10\n",
    )

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.obstacles == {
        "caja": SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05),
        "pared": SphereObstacle(center=Point(0.1, 0.4, 0.5), radius=0.10),
    }


def test_ignores_blank_lines_and_comments(tmp_path):
    path = _write(
        tmp_path / "obstacles.txt",
        "# obstaculos detectados\n\ncaja 0.3 0.0 0.2 0.05\n\n# fin\n",
    )

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.obstacles == {
        "caja": SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05),
    }


def test_repeated_name_keeps_the_last_line(tmp_path):
    path = _write(
        tmp_path / "obstacles.txt",
        "caja 0.0 0.0 0.0 0.05\ncaja 1.0 0.0 0.0 0.05\n",
    )

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.obstacles == {
        "caja": SphereObstacle(center=Point(1.0, 0.0, 0.0), radius=0.05),
    }


def test_parses_a_goal_line_into_the_fixed_objetivo_key(tmp_path):
    path = _write(tmp_path / "obstacles.txt", "objetivo 0.25 0.25 0.50\n")

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.objects == {"objetivo": Point(0.25, 0.25, 0.50)}
    assert scene.obstacles == {}


def test_goal_and_obstacles_coexist(tmp_path):
    path = _write(
        tmp_path / "obstacles.txt",
        "caja 0.3 0.0 0.2 0.05\nobjetivo 0.25 0.25 0.50\n",
    )

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.objects == {"objetivo": Point(0.25, 0.25, 0.50)}
    assert scene.obstacles == {"caja": SphereObstacle(center=Point(0.3, 0.0, 0.2), radius=0.05)}


def test_repeated_goal_line_keeps_the_last_one(tmp_path):
    path = _write(
        tmp_path / "obstacles.txt",
        "objetivo 0.0 0.0 0.0\nobjetivo 1.0 1.0 1.0\n",
    )

    scene = FilePerceptionAdapter(path).get_scene()

    assert scene.objects == {"objetivo": Point(1.0, 1.0, 1.0)}


def test_get_scene_reflects_the_current_content_of_the_file_each_call(tmp_path):
    path = tmp_path / "obstacles.txt"
    path.write_text("caja 0.0 0.0 0.0 0.05\n")
    adapter = FilePerceptionAdapter(str(path))

    before = adapter.get_scene()
    path.write_text("caja 0.0 0.0 0.0 0.05\nnueva 1.0 1.0 1.0 0.02\n")
    after = adapter.get_scene()

    assert list(before.obstacles) == ["caja"]
    assert list(after.obstacles) == ["caja", "nueva"]
