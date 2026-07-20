from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _compact(path):
    return " ".join(path.read_text(encoding="utf-8").split())


def test_cloud_stamp_publishers_use_reliable_safety_qos():
    cpp_adapter = _compact(
        WORKSPACE_SRC /
        "m20_sensor_pose_adapter_cpp/src/sensor_pose_adapter.cpp")
    python_adapter = _compact(
        PACKAGE_ROOT / "m20_scan_bringup/sensor_pose_adapter.py")

    assert "cloud_stamp_topic_, reliable_pair_qos" in cpp_adapter
    assert (
        "Header, self.cloud_stamp_topic, RELIABLE_SAFETY_QOS"
        in python_adapter
    )
    assert "ReliabilityPolicy.RELIABLE" in python_adapter


def test_both_safety_bridges_use_reliable_command_and_stamp_qos():
    bridge_paths = (
        PACKAGE_ROOT / "m20_scan_bringup/scan_m20_safety_bridge.py",
        PACKAGE_ROOT / "m20_scan_bringup/scan_m20_udp_safety_bridge.py",
    )

    for path in bridge_paths:
        bridge = _compact(path)
        assert "ReliabilityPolicy.RELIABLE" in bridge
        assert (
            "Twist, self.command_topic, self._command_callback, "
            "RELIABLE_SAFETY_QOS"
        ) in bridge
        assert (
            "Header, self.front_cloud_stamp_topic, "
            "self._front_cloud_callback, RELIABLE_SAFETY_QOS"
        ) in bridge


def test_controller_keeps_default_reliable_command_publisher():
    controller = _compact(
        WORKSPACE_SRC /
        "planner/plan_manage/src/closed_loop_controller.cpp")

    assert (
        'create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20)'
        in controller
    )
