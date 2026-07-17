from nav_msgs.msg import Odometry

import rclpy

from m20_scan_bringup.sensor_pose_from_odom_adapter import (
    SensorPoseFromOdomAdapter,
)


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_body_pose_callback_publishes_matching_header():
    rclpy.init()
    node = SensorPoseFromOdomAdapter()
    capture = CapturingPublisher()
    node.body_pose_stamp_pub = capture
    try:
        odom = Odometry()
        odom.header.stamp.sec = 123
        odom.header.stamp.nanosec = 456
        odom.header.frame_id = "world"

        node.body_pose_callback(odom)

        assert len(capture.messages) == 1
        header = capture.messages[0]
        assert header.stamp == odom.header.stamp
        assert header.frame_id == "world"
        assert node.body_poses[-1] is odom
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_body_pose_callback_rate_limits_stamp_but_keeps_pose_history():
    rclpy.init()
    node = SensorPoseFromOdomAdapter()
    capture = CapturingPublisher()
    node.body_pose_stamp_pub = capture
    node.body_stamp_period_sec = 10.0
    try:
        first = Odometry()
        first.header.frame_id = "world"
        second = Odometry()
        second.header.frame_id = "world"

        node.body_pose_callback(first)
        node.body_pose_callback(second)

        assert len(capture.messages) == 1
        poses = list(node.body_poses)
        assert poses[-2] is first
        assert poses[-1] is second
    finally:
        node.destroy_node()
        rclpy.shutdown()
