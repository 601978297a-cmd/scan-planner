from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_uses_super_lio_for_every_body_pose_consumer():
    launch = _read("launch/m20_scan_dry_run.launch.py")

    assert launch.count('"/lio/robo/odom"') == 3
    assert "/lightning/odom" not in launch
    assert '"target_frame": "world"' in launch
    assert "map_to_world_for_scan_markers" not in launch


def test_planner_keeps_front_cloud_in_sensor_coordinates():
    planner = _read("config/m20_scan_planner.yaml")

    assert "grid_map.frame_id: world" in planner
    assert "grid_map.cloud_is_world: false" in planner
    assert "grid_map.need_extrinsic: false" in planner


def test_operator_and_legacy_safety_configs_use_super_lio():
    rviz = _read("rviz/m20_scan.rviz")
    safety = _read("config/m20_scan_safety_bridge.yaml")

    assert "Fixed Frame: world" in rviz
    assert "Value: /lio/robo/odom" in rviz
    assert "body_pose_topic: /lio/robo/odom" in safety


def test_rviz_defaults_to_low_load_navigation_view():
    rviz = _read("rviz/m20_scan.rviz")

    raw_cloud_name = rviz.index("Name: Front LiDAR Raw Cloud")
    raw_cloud_start = rviz.rfind("    - Alpha:", 0, raw_cloud_name)
    occupancy_start = rviz.index("Name: Occupancy")
    raw_cloud = rviz[raw_cloud_start:occupancy_start]

    assert "Enabled: false" in raw_cloud
    assert "Value: false" in raw_cloud
    assert "Frame Rate: 10" in rviz
    assert "Reference Frame: world" in rviz
