from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage

from m20_scan_bringup.smac_to_scan_bridge import sanitize_path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def _read_source(relative_path):
    return (SOURCE_ROOT / relative_path).read_text(encoding="utf-8")


def _pose(x, y, z=1.0, frame_id="world"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def test_smac_path_is_spaced_and_converted_for_scan():
    path = PathMessage()
    path.header.frame_id = "world"
    path.poses = [
        _pose(0.0, 0.0),
        _pose(0.1, 0.0),
        _pose(0.25, 0.0),
        _pose(0.5, 0.0),
    ]

    sanitized = sanitize_path(path, "world", 0.20)

    assert sanitized is not None
    assert [pose.pose.position.x for pose in sanitized.poses] == [0.25, 0.5]
    assert all(pose.pose.position.z == 0.0 for pose in sanitized.poses)
    assert all(
        pose.header.frame_id == "world" for pose in sanitized.poses)


def test_smac_path_rejects_wrong_frame_and_non_finite_points():
    wrong_frame = PathMessage()
    wrong_frame.header.frame_id = "map"
    wrong_frame.poses = [_pose(0.0, 0.0), _pose(1.0, 0.0)]
    assert sanitize_path(wrong_frame, "world", 0.20) is None

    invalid = PathMessage()
    invalid.header.frame_id = "world"
    invalid.poses = [_pose(0.0, 0.0), _pose(float("nan"), 0.0)]
    assert sanitize_path(invalid, "world", 0.20) is None


def test_mode3_launch_starts_smac_and_keeps_scan_mode_runtime_selectable():
    mode3_launch = _read("launch/m20_mode3_navigation.launch.py")
    scan_launch = _read("launch/m20_scan_dry_run.launch.py")
    planner_config = _read("config/m20_scan_planner.yaml")
    smac_config = _read("config/m20_smac_global.yaml")
    map_config = _read("config/m20_mode3_map.yaml")

    assert 'package="nav2_map_server"' in mode3_launch
    assert 'package="nav2_planner"' in mode3_launch
    assert 'package="nav2_lifecycle_manager"' in mode3_launch
    assert 'executable="smac_to_scan_bridge"' in mode3_launch
    assert '"navi_mode": "3"' in mode3_launch
    assert (
        "/home/nvidia/Super-LIO/src/super_lio/map/map.yaml"
        in mode3_launch
    )
    assert 'default_value="1"' in scan_launch
    assert '"fsm.navi_mode": ParameterValue(' in scan_launch
    assert "fsm.navi_mode: 1" in planner_config
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in smac_config
    assert "downsample_costmap: false" in smac_config
    assert "downsampling_factor: 1" in smac_config
    assert "resolution: 0.20" in smac_config
    assert "global_frame: world" in smac_config
    assert "robot_base_frame: base_link_dog" in smac_config
    assert "robot_radius: 0.30" in smac_config
    assert "mode: trinary" in map_config
    assert "resolution: 0.20000000298023224" in map_config
    assert (
        "origin: [-20.892608928618962, -52.00981577417834, 0.0]"
        in map_config
    )
    assert "free_thresh: 0.196" in map_config
    assert (
        "image: /home/nvidia/Super-LIO/src/super_lio/map/map.pgm"
        in map_config
    )


def test_mode3_replan_preserves_reference_path():
    source = _read_source(
        "planner/plan_manage/src/scan_replan_fsm.cpp")

    reference_guard = source.index(
        "if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)",
        source.index("bool SCANReplanFSM::planFromCurrentTraj()"))
    refresh_global = source.index(
        "planner_manager_->planGlobalTraj(", reference_guard)

    assert reference_guard < refresh_global
    assert "callReboundReplan(false, false)" in source[
        reference_guard:refresh_global]
    assert "std::numeric_limits<double>::max()" in source
    assert "global_data.last_progress_time_ = projection_t;" in source


def test_mode3_startup_is_separate_from_mode1_startup():
    mode1_script = (
        REPOSITORY_ROOT / "scripts/start_m20_navigation.sh"
    ).read_text(encoding="utf-8")
    mode3_script = (
        REPOSITORY_ROOT / "scripts/start_m20_mode3_navigation.sh"
    ).read_text(encoding="utf-8")

    assert "m20_mode3_navigation.launch.py" not in mode1_script
    assert "m20_scan_dry_run.launch.py" in mode1_script
    assert "m20_mode3_navigation.launch.py" in mode3_script
    assert (
        "/home/nvidia/Super-LIO/src/super_lio/map/map.yaml"
        in mode3_script
    )
    assert "/home/nvidia/Super-LIO/src/super_lio/map/map.pgm" in mode3_script


def test_rviz_shows_mode3_map_and_global_paths():
    rviz_config = _read("rviz/m20_scan.rviz")

    assert "Name: 2D Global Map" in rviz_config
    assert "Value: /map" in rviz_config
    assert "Name: Smac Global Path" in rviz_config
    assert "Value: /plan" in rviz_config
    assert "Name: SCAN Reference Path" in rviz_config
    assert "Value: /initial_path" in rviz_config
    assert "Name: SCAN Local Rebound A Star" in rviz_config
