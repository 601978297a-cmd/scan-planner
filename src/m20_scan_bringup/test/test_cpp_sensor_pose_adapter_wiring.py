from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _read(path):
    return path.read_text(encoding="utf-8")


def test_cpp_adapter_is_default_with_python_fallback():
    launch = _read(PACKAGE_ROOT / "launch/m20_scan_dry_run.launch.py")

    assert '"sensor_pose_adapter_backend"' in launch
    assert 'default_value="cpp"' in launch
    assert 'choices=["cpp", "python"]' in launch
    assert 'package="m20_sensor_pose_adapter_cpp"' in launch
    assert 'executable="sensor_pose_adapter_cpp"' in launch
    assert 'package="m20_scan_bringup"' in launch
    assert 'executable="sensor_pose_adapter"' in launch


def test_cpp_adapter_preserves_topics_and_three_execution_paths():
    source = _read(
        WORKSPACE_SRC /
        "m20_sensor_pose_adapter_cpp/src/sensor_pose_adapter.cpp")

    assert '"/rslidar_points_front"' in source
    assert '"/scan/sensor_pose"' in source
    assert '"/scan/front_cloud_stamp"' in source
    assert '"/scan/front_cloud_synced"' in source
    assert "tf_buffer_, this, true" in source
    assert "cloud_options.callback_group = cloud_callback_group_" in source
    assert "match_callback_group_" in source
    assert "MultiThreadedExecutor" in source
    assert "ExecutorOptions(), 2" in source


def test_cpp_adapter_and_planner_use_reliable_paired_cloud_qos():
    adapter = _read(
        WORKSPACE_SRC /
        "m20_sensor_pose_adapter_cpp/src/sensor_pose_adapter.cpp")
    planner = _read(
        WORKSPACE_SRC /
        "planner/plan_env/src/grid_map.cpp")

    assert "rclcpp::KeepLast(5)).reliable().durability_volatile()" in adapter
    assert "cloud_topic_,\n      reliable_pair_qos" in adapter
    assert "synced_cloud_topic_, reliable_pair_qos" in adapter
    assert "output_topic_, reliable_pair_qos" in adapter
    assert "rmw_qos_profile_sensor_data" in planner
    assert (
        "reliable_pair_qos.reliability = "
        "RMW_QOS_POLICY_RELIABILITY_RELIABLE"
    ) in planner
    assert "reliable_pair_qos.depth = 5" in planner
    assert (
        'cloud_sub_->subscribe(node_, "cloud", reliable_pair_qos)'
    ) in planner
    assert (
        'lidar_pose_sub_->subscribe(node_, "sensor_pose", reliable_pair_qos)'
    ) in planner
