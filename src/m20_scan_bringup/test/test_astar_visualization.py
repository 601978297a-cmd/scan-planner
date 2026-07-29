from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def test_local_astar_visualization_is_stable_and_removes_stale_paths():
    source = (
        REPOSITORY_ROOT
        / "planner/traj_utils/src/planning_visualization.cpp"
    ).read_text(encoding="utf-8")

    function = source[
        source.index("void PlanningVisualization::displayAStarList"):
        source.index("void PlanningVisualization::displayArrowList")
    ]

    assert "rand()" not in function
    assert "constexpr double scale = 0.06" in function
    assert "visualization_msgs::msg::Marker::DELETE" in function
    assert "last_a_star_path_count_ = i" in function
