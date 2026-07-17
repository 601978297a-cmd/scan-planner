from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

import rclpy

from m20_scan_bringup.sensor_pose_adapter import SensorPoseAdapter


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FixedTransformBuffer:
    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, *_args, **_kwargs):
        return self.transform


class NoopTransformListener:
    def __init__(self, *_args, **_kwargs):
        pass


def test_cloud_callback_publishes_stamp_and_tf_pose(monkeypatch):
    monkeypatch.setattr(
        "m20_scan_bringup.sensor_pose_adapter.tf2_ros.TransformListener",
        NoopTransformListener,
    )
    rclpy.init()
    node = SensorPoseAdapter()
    pose_capture = CapturingPublisher()
    stamp_capture = CapturingPublisher()
    node.pose_pub = pose_capture
    node.cloud_stamp_pub = stamp_capture
    try:
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "rslidar_front"
        transform.transform.translation.x = 0.32
        transform.transform.rotation.w = 1.0
        node.tf_buffer = FixedTransformBuffer(transform)

        cloud = PointCloud2()
        cloud.header.stamp.sec = 123
        cloud.header.stamp.nanosec = 456
        cloud.header.frame_id = "rslidar_front"
        node.cloud_callback(cloud)

        assert len(stamp_capture.messages) == 1
        assert stamp_capture.messages[0].stamp == cloud.header.stamp
        assert stamp_capture.messages[0].frame_id == "rslidar_front"

        assert len(pose_capture.messages) == 1
        pose = pose_capture.messages[0]
        assert pose.header.stamp == cloud.header.stamp
        assert pose.header.frame_id == "map"
        assert pose.child_frame_id == "rslidar_front"
        assert pose.pose.pose.position.x == 0.32
        assert pose.pose.pose.orientation.w == 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
