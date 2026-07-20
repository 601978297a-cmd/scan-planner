from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import SetBool

import rclpy

from m20_scan_bringup.scan_m20_udp_safety_bridge import (
    ScanM20UdpSafetyBridge,
)


class FailingUdpLink:
    def send_axis(self, _axis):
        raise OSError("test send failure")

    def close(self):
        pass


def test_default_udp_node_rejects_arm_without_opening_socket():
    rclpy.init()
    node = ScanM20UdpSafetyBridge()
    try:
        assert not node.enable_udp_output
        assert node.stop_cycles == 20
        assert node.udp_link is None
        request = SetBool.Request()
        request.data = True
        response = node._handle_arm(request, SetBool.Response())
        assert not response.success
        assert "enable_udp_output_false" in response.message
        assert node.udp_link is None
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
