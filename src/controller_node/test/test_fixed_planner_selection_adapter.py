from shared_kernel import Point, Scene

from controller_node.adapters.fixed_planner_selection_adapter import (
    FixedPlannerSelectionAdapter,
)


def test_always_returns_the_configured_strategy_regardless_of_scene():
    adapter = FixedPlannerSelectionAdapter("straight_line")
    populated_scene = Scene.empty().with_object("cup", Point(0.1, 0.2, 0.3))

    assert adapter.select(Scene.empty()) == "straight_line"
    assert adapter.select(populated_scene) == "straight_line"
