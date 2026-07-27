from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


def _read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def _read_source(relative_path):
    return (SOURCE_ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_uses_super_lio_for_every_body_pose_consumer():
    launch = _read("launch/m20_scan_dry_run.launch.py")

    assert launch.count('"/lio/robo/odom"') == 2
    assert "/lightning/odom" not in launch
    assert 'executable="sensor_pose_adapter"' in launch
    assert '"target_frame": "world"' in launch
    assert "map_to_world_for_scan_markers" not in launch


def test_planner_uses_cloud_republished_with_matching_sensor_pose():
    launch = _read("launch/m20_scan_dry_run.launch.py")

    assert '"synced_cloud_topic": "/scan/front_cloud_synced"' in launch
    assert '("cloud", "/scan/front_cloud_synced")' in launch
    assert '"max_tf_wait_sec": 0.5' in launch
    assert '"max_queue_size": 2' in launch


def test_planner_keeps_front_cloud_in_sensor_coordinates():
    planner = _read("config/m20_scan_planner.yaml")

    assert "grid_map.frame_id: world" in planner
    assert "grid_map.cloud_is_world: false" in planner
    assert "grid_map.need_extrinsic: false" in planner
    assert "grid_map.voxel_leaf_size: 0.10" in planner
    assert "grid_map.resolution: 0.10" in planner


def test_planner_and_controller_use_matching_forward_speed_limit():
    planner = _read("config/m20_scan_planner.yaml")
    controller = _read("config/m20_scan_controller.yaml")

    assert "manager.max_vel: 0.30" in planner
    assert "optimization.max_vel: 0.30" in planner
    assert "manager.max_acc: 0.20" in planner
    assert "optimization.max_acc: 0.20" in planner
    assert "max_vx: 0.30" in controller
    assert "max_accel: 0.20" in controller
    assert "max_decel: 0.80" in controller
    assert "max_yaw_accel: 0.40" in controller
    assert "max_yaw_decel: 1.00" in controller


def test_closed_loop_controller_smooths_normal_velocity_commands():
    controller = _read_source(
        "planner/plan_manage/src/closed_loop_controller.cpp")

    assert "slewLimit(" in controller
    assert "publishSmoothedCommand(" in controller
    assert "last_command_" in controller
    assert "publishSmoothedCommand(command, dt);" in controller
    assert "publishStop();" in controller


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


def test_m20_navigation_is_gated_by_udp_arm_state():
    launch = _read("launch/m20_scan_dry_run.launch.py")
    planner_config = _read("config/m20_scan_planner.yaml")
    controller_config = _read("config/m20_scan_controller.yaml")
    udp_config = _read("config/m20_scan_udp_bridge.yaml")
    fsm_source = _read_source(
        "planner/plan_manage/src/scan_replan_fsm.cpp")
    controller_source = _read_source(
        "planner/plan_manage/src/closed_loop_controller.cpp")

    assert "fsm.require_navigation_enable: true" in planner_config
    assert "require_navigation_enable: true" in controller_config
    assert "navigation_enabled_topic: /scan/navigation_enabled" in udp_config
    assert "auto_arm_enabled: true" in udp_config
    assert "auto_arm_stable_sec: 2.0" in udp_config
    assert "yaw_zero_epsilon: 0.04" in udp_config
    assert "udp_yaw_slew_rate: 2.0" in udp_config
    assert '"enable_udp_output"' in launch
    assert "ParameterValue(" in launch
    assert "navigationEnabledCallback" in fsm_source
    assert "cancelNavigation" in fsm_source
    assert "navigationEnabledCallback" in controller_source
    assert "clearTrajectory" in controller_source


def test_m20_collision_and_udp_mapping_match_approved_design():
    planner_config = _read("config/m20_scan_planner.yaml")
    udp_config = _read("config/m20_scan_udp_bridge.yaml")

    assert "grid_map.double_cylinder_radius: 0.30" in planner_config
    assert "grid_map.double_cylinder_offset: 0.20" in planner_config
    assert "optimization.dist0: 0.25" in planner_config

    assert "enable_udp_output: false" in udp_config
    assert "command_rate: 20.0" in udp_config
    assert "preview_rate: 20.0" in udp_config
    assert "max_vx: 0.15" in udp_config
    assert "max_wz: 0.20" in udp_config
    assert "udp_max_x: 0.50" in udp_config
    assert "udp_max_yaw: 0.60" in udp_config
    assert "udp_yaw_deadzone" not in udp_config
    assert "yaw_start_threshold" not in udp_config
    assert "yaw_stop_threshold" not in udp_config


def test_optimized_trajectory_marker_uses_map_and_transient_local_qos():
    rviz = _read("rviz/m20_scan.rviz")
    marker_name = rviz.index("Name: Optimized Trajectory")
    marker_start = rviz.rfind("    - Class:", 0, marker_name)
    marker_end = rviz.index("    - Class:", marker_name)
    marker = rviz[marker_start:marker_end]
    visualization_source = _read_source(
        "planner/traj_utils/src/planning_visualization.cpp")
    fsm_source = _read_source(
        "planner/plan_manage/src/scan_replan_fsm.cpp")

    assert "Durability Policy: Transient Local" in marker
    assert 'header.frame_id = "world"' not in visualization_source
    assert 'header.frame_id = "map"' not in visualization_source
    assert "new PlanningVisualization(node_, self_inflation_frame_id_)" in fsm_source


def test_removed_filter_and_odom_composition_adapters_are_not_entry_points():
    setup = _read("setup.py")

    assert "cloud_filter_adapter" not in setup
    assert "sensor_pose_from_odom_adapter" not in setup
