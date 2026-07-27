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

    assert "rclcpp::KeepLast(1)).reliable().durability_volatile()" in adapter
    assert "cloud_topic_,\n      reliable_pair_qos" in adapter
    assert "synced_cloud_topic_, reliable_pair_qos" in adapter
    assert "output_topic_, reliable_pair_qos" in adapter
    assert "rmw_qos_profile_sensor_data" in planner
    assert (
        "reliable_pair_qos.reliability = "
        "RMW_QOS_POLICY_RELIABILITY_RELIABLE"
    ) in planner
    assert "reliable_pair_qos.depth = 1" in planner
    assert "SyncPolicyCloudPose(1)" in planner
    assert (
        'node_, "cloud", reliable_pair_qos, mapping_options'
    ) in planner
    assert (
        'node_, "sensor_pose", reliable_pair_qos, mapping_options'
    ) in planner


def test_planner_voxel_filters_cloud_before_raycasting():
    planner = _read(
        WORKSPACE_SRC /
        "planner/plan_env/src/grid_map.cpp")

    filter_call = planner.index("voxel_filter.filter(latest_cloud);")
    raycast_loop = planner.index(
        "for (size_t i = 0; i < latest_cloud.points.size(); ++i)")

    assert '#include <pcl/filters/voxel_grid.h>' in planner
    assert '"grid_map.voxel_leaf_size"' in planner
    assert filter_call < raycast_loop


def test_planner_separates_mapping_from_planning_callbacks():
    node = _read(
        WORKSPACE_SRC /
        "planner/plan_manage/src/scan_planner_node.cpp")
    fsm = _read(
        WORKSPACE_SRC /
        "planner/plan_manage/src/scan_replan_fsm.cpp")
    grid_map = _read(
        WORKSPACE_SRC /
        "planner/plan_env/src/grid_map.cpp")

    assert "MultiThreadedExecutor" in node
    assert "ExecutorOptions(), 2" in node
    assert "planning_callback_group_" in fsm
    assert "mapping_callback_group_" in fsm
    assert "CallbackGroupType::MutuallyExclusive" in fsm
    assert "planning_options.callback_group = planning_callback_group_" in fsm
    assert "mapping_options.callback_group = mapping_callback_group" in grid_map
    assert "mapping_callback_group);" in grid_map


def test_planner_reads_immutable_map_snapshot_and_limits_visualization_work():
    grid_map = _read(
        WORKSPACE_SRC /
        "planner/plan_env/src/grid_map.cpp")
    config = _read(PACKAGE_ROOT / "config/m20_scan_planner.yaml")

    assert "std::atomic_store_explicit" in grid_map
    assert "std::atomic_load_explicit" in grid_map
    assert "publishPlanningSnapshot();" in grid_map
    assert "publishMaps(true, true);" in grid_map
    assert "occupancy_version_ != last_visualized_version_" in grid_map
    assert "grid_map.visualization_period_ms: 200" in config


def test_cpp_adapter_publishes_stamp_before_pose_and_cloud():
    source = _read(
        WORKSPACE_SRC /
        "m20_sensor_pose_adapter_cpp/src/sensor_pose_adapter.cpp")

    stamp_publish = source.index(
        "cloud_stamp_pub_->publish(cloud_stamp);")
    pose_publish = source.index("pose_pub_->publish(pose);")
    cloud_publish = source.index("synced_cloud_pub_->publish(*cloud);")

    assert stamp_publish < pose_publish < cloud_publish
