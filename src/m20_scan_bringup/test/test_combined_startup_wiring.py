from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _read(path):
    return path.read_text(encoding="utf-8")


def test_combined_launch_starts_relocation_and_direct_udp_backend():
    launch = _read(
        PACKAGE_ROOT / "launch/m20_localization_navigation.launch.py")

    assert 'get_package_share_directory("super_lio")' in launch
    assert '"relocation.py"' in launch
    assert '"rviz": "false"' in launch
    assert '"m20_scan_dry_run.launch.py"' in launch
    assert '"control_backend": "direct_udp"' in launch
    assert '"enable_nav_cmd_output": "false"' in launch
    assert '"enable_udp_output": "false"' in launch
    assert '"require_navigation_enable": "false"' in launch


def test_direct_udp_backend_uses_scan_command_topic_and_super_lio_mapping():
    dry_run_launch = _read(
        PACKAGE_ROOT / "launch/m20_scan_dry_run.launch.py")
    config = _read(
        PACKAGE_ROOT / "config/m20_scan_direct_udp.yaml")
    setup = _read(PACKAGE_ROOT / "setup.py")

    assert '"direct_udp"' in dry_run_launch
    assert 'executable="scan_m20_direct_udp_bridge"' in dry_run_launch
    assert '"require_navigation_enable"' in dry_run_launch
    assert '"fsm.require_navigation_enable": ParameterValue(' in dry_run_launch
    assert '"require_navigation_enable": ParameterValue(' in dry_run_launch
    assert "command_topic: /scan/cmd_vel_debug" in config
    assert "udp_target_host: 10.21.31.103" in config
    assert "udp_target_port: 30000" in config
    assert "scale_x: 3.0" in config
    assert "scale_y: 3.0" in config
    assert "scale_yaw: 2.0" in config
    assert "timeout_ms: 500.0" in config
    assert "send_stand_on_start: false" in config
    assert "scan_m20_direct_udp_bridge =" in setup


def test_nav_cmd_backend_has_independent_real_output_gate():
    dry_run_launch = _read(
        PACKAGE_ROOT / "launch/m20_scan_dry_run.launch.py")
    config = _read(
        PACKAGE_ROOT / "config/m20_scan_safety_bridge.yaml")
    package = _read(PACKAGE_ROOT / "package.xml")

    assert '"enable_nav_cmd_output"' in dry_run_launch
    assert '"enable_m20_output": ParameterValue(' in dry_run_launch
    assert "enable_m20_output: false" in config
    assert "auto_arm_enabled: true" in config
    assert "auto_arm_stable_sec: 2.0" in config
    assert "stop_cycles: 20" in config
    assert "forward_speed: 0.15" in config
    assert "yaw_speed: 0.35" in config
    assert "motion_info_topic: /MOTION_INFO" in config
    assert "navigation_enabled_topic: /scan/navigation_enabled" in config
    assert "<exec_depend>drdds</exec_depend>" in package
    assert "<exec_depend>scan_planner</exec_depend>" in package


def test_scan_workspace_contains_required_drdds_interfaces():
    drdds = REPOSITORY_ROOT / "src/drdds"

    for relative_path in (
        "package.xml",
        "CMakeLists.txt",
        "msg/MetaType.msg",
        "msg/NavCmdValue.msg",
        "msg/NavCmd.msg",
        "msg/MotionStateValue.msg",
        "msg/GaitValue.msg",
        "msg/MotionInfoValue.msg",
        "msg/MotionInfo.msg",
    ):
        assert (drdds / relative_path).is_file()


def test_combined_startup_script_uses_only_version1_workspaces():
    script = _read(
        REPOSITORY_ROOT / "scripts/start_m20_localization_navigation.sh")

    assert (
        "/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
        in script
    )
    assert (
        "/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"
        in script
    )
    assert "/home/nvidia/Super-LIO/install/setup.bash" not in script
    assert "m20_localization_navigation.launch.py" in script


def test_rviz_script_starts_only_scan_rviz():
    script = _read(REPOSITORY_ROOT / "scripts/start_m20_rviz.sh")

    assert (
        "/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
        in script
    )
    assert (
        "/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"
        in script
    )
    assert "m20_scan.rviz" in script
    assert "ros2 launch" not in script
