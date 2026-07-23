from types import SimpleNamespace

from std_srvs.srv import SetBool

import rclpy
from std_msgs.msg import Header

from m20_scan_bringup.scan_m20_safety_bridge import ScanM20SafetyBridge
from m20_scan_bringup.safety_bridge_core import BridgeState, Command


def test_default_node_rejects_arm_without_creating_nav_publisher():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.nav_pub is None
        request = SetBool.Request()
        request.data = True
        response = node._handle_arm(request, SetBool.Response())
        assert not response.success
        assert "enable_m20_output_false" in response.message
        assert node.nav_pub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_uses_lightweight_front_cloud_stamp():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.front_cloud_stamp_topic == "/scan/front_cloud_stamp"
        assert node.front_cloud_sub.topic_name == "/scan/front_cloud_stamp"
        assert node.front_cloud_sub.msg_type is Header

        stamp = Header()
        stamp.stamp.sec = 12
        stamp.stamp.nanosec = 345
        node._front_cloud_callback(stamp)

        assert node.inputs.cloud_rx is not None
        assert node.inputs.cloud_stamp_ns == 12_000_000_345
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_motion_info_tracks_state_gait_and_finite_feedback():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        msg = SimpleNamespace(data=SimpleNamespace(
            vel_x=0.0,
            vel_y=0.0,
            vel_yaw=0.0,
            motion_state=SimpleNamespace(state=17),
            gait_state=SimpleNamespace(gait=0x3002),
        ))

        node._motion_info_callback(msg)

        assert node.motion_state == 17
        assert node.gait_state == 0x3002
        assert node.motion_info_finite is True
        assert node._robot_state_reasons() == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_robot_state_reasons_fail_closed():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert "motion_state_not_rl" in node._robot_state_reasons()
        assert "gait_not_agile_flat" in node._robot_state_reasons()

        node.motion_state = 17
        node.gait_state = 0x3002
        node.motion_info_finite = False
        assert node._robot_state_reasons() == ["motion_info_nonfinite"]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_auto_arm_waits_for_stable_window(monkeypatch):
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.enable_m20_output = True
        node.auto_arm_stable_sec = 2.0
        armed_from = []
        monkeypatch.setattr(
            node,
            "_arm_bridge",
            lambda source: armed_from.append(source) or True,
        )

        assert not node._update_auto_arm(10.0, [])
        assert not node._update_auto_arm(11.9, [])
        assert node._update_auto_arm(12.0, [])
        assert armed_from == ["automatic"]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_disarm_latches_and_blocks_auto_arm():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.enable_m20_output = True
        request = SetBool.Request()
        request.data = False

        response = node._handle_arm(request, SetBool.Response())

        assert response.success
        assert node.manual_disarm_latched is True
        assert not node._update_auto_arm(10.0, [])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_begin_stop_revokes_navigation_and_clears_preview():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.state_machine.arm()
        node.navigation_enabled = True
        node.preview_command = Command(vx=0.15, wz=0.35)

        node._begin_stop("test_stop")

        assert node.navigation_enabled is False
        assert node.preview_command == Command()
        assert node.state_machine.state is BridgeState.STOPPING
        assert node.state_machine.stop_cycles_remaining == 20
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_publish_failure_aborts_output_and_requires_manual_rearm():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.state_machine.arm()
        node.navigation_enabled = True

        node._abort_output("nav_cmd_publish_failed")

        assert node.state_machine.state is BridgeState.DISARMED
        assert node.navigation_enabled is False
        assert node.manual_disarm_latched is True
        assert node.stop_reason == "nav_cmd_publish_failed"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_existing_nav_cmd_publisher_blocks_arm(monkeypatch):
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        node.enable_m20_output = True
        monkeypatch.setattr(node, "_initialize_drdds_runtime", lambda: True)
        monkeypatch.setattr(node, "_health_reasons", lambda **_kwargs: [])
        monkeypatch.setattr(node, "_robot_state_reasons", lambda: [])
        monkeypatch.setattr(node, "count_publishers", lambda _topic: 1)

        assert "nav_cmd_publisher_exists" in node._arm_blockers()
    finally:
        node.destroy_node()
        rclpy.shutdown()
