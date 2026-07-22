from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import SetBool

import rclpy

from m20_scan_bringup.scan_m20_udp_safety_bridge import (
    ScanM20UdpSafetyBridge,
)
from m20_scan_bringup.m20_udp_protocol import UdpAxis
from m20_scan_bringup.safety_bridge_core import BridgeState


class FailingUdpLink:
    def send_axis(self, _axis):
        raise OSError("test send failure")

    def close(self):
        pass


class CapturingUdpLink:
    def __init__(self):
        self.axes = []

    def send_axis(self, axis):
        self.axes.append(axis)

    def close(self):
        pass


def test_default_udp_node_rejects_arm_without_opening_socket():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        assert not node.enable_udp_output
        assert node.stop_cycles == 20
        assert node.udp_link is None
        assert node.navigation_enabled is False
        request = SetBool.Request()
        request.data = True
        response = node._handle_arm(request, SetBool.Response())
        assert not response.success
        assert "enable_udp_output_false" in response.message
        assert node.udp_link is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_navigation_enable_tracks_arm_and_disarm():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node._arm_blockers = lambda: []

        arm_request = SetBool.Request()
        arm_request.data = True
        arm_response = node._handle_arm(arm_request, SetBool.Response())
        assert arm_response.success
        assert node.state_machine.state is BridgeState.ARMED
        assert node.navigation_enabled is True

        disarm_request = SetBool.Request()
        disarm_request.data = False
        disarm_response = node._handle_arm(
            disarm_request, SetBool.Response())
        assert disarm_response.success
        assert node.state_machine.state is BridgeState.STOPPING
        assert node.navigation_enabled is False
        assert node.manual_disarm_latched is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_auto_arm_waits_for_continuous_stability_window():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.auto_arm_stable_sec = 2.0
        node.enable_udp_output = True

        assert not node._update_auto_arm(10.0, [])
        assert not node._update_auto_arm(11.9, [])
        assert node.state_machine.state is BridgeState.DISARMED
        assert node._update_auto_arm(12.0, [])
        assert node.state_machine.state is BridgeState.ARMED
        assert node.navigation_enabled is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_auto_arm_blocker_resets_stability_window():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.auto_arm_stable_sec = 2.0
        node.enable_udp_output = True

        assert not node._update_auto_arm(10.0, [])
        assert not node._update_auto_arm(11.0, ["body_pose_stale"])
        assert node.auto_arm_ready_since is None
        assert not node._update_auto_arm(12.0, [])
        assert not node._update_auto_arm(13.9, [])
        assert node._update_auto_arm(14.0, [])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_disarm_latch_prevents_auto_arm():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.enable_udp_output = True

        request = SetBool.Request()
        request.data = False
        response = node._handle_arm(request, SetBool.Response())
        assert response.success
        assert node.manual_disarm_latched is True
        assert not node._update_auto_arm(10.0, [])
        assert not node._update_auto_arm(20.0, [])
        assert node.state_machine.state is BridgeState.DISARMED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_only_successful_manual_arm_clears_manual_latch():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.manual_disarm_latched = True
        node._arm_blockers = lambda: ["body_pose_stale"]
        request = SetBool.Request()
        request.data = True
        response = node._handle_arm(request, SetBool.Response())
        assert not response.success
        assert node.manual_disarm_latched is True

        node._arm_blockers = lambda: []
        response = node._handle_arm(request, SetBool.Response())
        assert response.success
        assert node.manual_disarm_latched is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_automatic_stop_can_rearm_without_restoring_manual_permission():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.auto_arm_stable_sec = 2.0
        node.enable_udp_output = True
        assert not node._update_auto_arm(10.0, [])
        assert node._update_auto_arm(12.0, [])

        node.stop_cycles = 1
        node._begin_stop("body_pose_stale")
        assert node.navigation_enabled is False
        assert node.manual_disarm_latched is False
        assert node.state_machine.record_stop_publish()
        assert node.state_machine.state is BridgeState.DISARMED

        assert not node._update_auto_arm(20.0, [])
        assert node._update_auto_arm(22.0, [])
        assert node.navigation_enabled is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dry_run_never_auto_arms():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.auto_arm_enabled = True
        node.auto_arm_stable_sec = 0.0
        assert not node.enable_udp_output
        assert not node._update_auto_arm(10.0, [])
        assert node.state_machine.state is BridgeState.DISARMED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failed_zero_send_does_not_advance_stopping_sequence():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        node.udp_link = FailingUdpLink()
        assert node.state_machine.arm()
        node.state_machine.begin_stop(2)
        node._tick()
        assert node.state_machine.stop_cycles_remaining == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stopping_bypasses_yaw_slew_and_sends_zero_immediately():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        link = CapturingUdpLink()
        node.udp_link = link
        node.preview_axis = UdpAxis(yaw=0.50)
        node.stop_cycles = 1
        assert node.state_machine.arm()
        node._begin_stop("test_stop")

        assert node.preview_axis == UdpAxis()

        node._tick()

        assert link.axes == [UdpAxis()]
        assert node.preview_axis == UdpAxis()
        assert node.state_machine.state is BridgeState.DISARMED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_new_cloud_does_not_invalidate_pose_matching_previous_cloud():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        first_cloud = Header()
        first_cloud.stamp.sec = 1
        node._front_cloud_callback(first_cloud)

        sensor_pose = Odometry()
        sensor_pose.header.stamp.sec = 1
        node._sensor_pose_callback(sensor_pose)

        second_cloud = Header()
        second_cloud.stamp.sec = 1
        second_cloud.stamp.nanosec = 200_000_000
        node._front_cloud_callback(second_cloud)

        reasons = node._health_reasons(node.inputs.cloud_rx)
        assert "sensor_cloud_stamp_mismatch" not in reasons
        assert "sensor_cloud_stamp_missing" not in reasons
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pose_before_cloud_completes_timestamp_pair():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        sensor_pose = Odometry()
        sensor_pose.header.stamp.sec = 2
        node._sensor_pose_callback(sensor_pose)

        front_cloud = Header()
        front_cloud.stamp.sec = 2
        node._front_cloud_callback(front_cloud)

        reasons = node._health_reasons(node.inputs.cloud_rx)
        assert "sensor_cloud_stamp_mismatch" not in reasons
        assert "sensor_cloud_stamp_missing" not in reasons
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_body_pose_odom_updates_freshness():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        assert node.inputs.body_pose_rx is None
        node._body_pose_callback(Odometry())
        assert node.inputs.body_pose_rx is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()
