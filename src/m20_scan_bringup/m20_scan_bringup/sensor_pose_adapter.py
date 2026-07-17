#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros


@dataclass
class PendingCloud:
    message: PointCloud2
    received_at: float


class SensorPoseAdapter(Node):
    def __init__(self):
        super().__init__("sensor_pose_adapter")
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.source_frame = self.declare_parameter(
            "source_frame", "rslidar_front").value
        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/rslidar_points_front").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/scan/sensor_pose").value
        self.cloud_stamp_topic = self.declare_parameter(
            "cloud_stamp_topic", "/scan/front_cloud_stamp").value
        self.synced_cloud_topic = self.declare_parameter(
            "synced_cloud_topic", "/scan/front_cloud_synced").value
        self.max_tf_wait_sec = float(
            self.declare_parameter("max_tf_wait_sec", 0.5).value)
        self.retry_period_sec = float(
            self.declare_parameter("retry_period_sec", 0.02).value)
        self.max_queue_size = int(
            self.declare_parameter("max_queue_size", 16).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.pose_pub = self.create_publisher(
            Odometry, self.output_topic, qos_profile_sensor_data)
        self.cloud_stamp_pub = self.create_publisher(
            Header, self.cloud_stamp_topic, qos_profile_sensor_data)
        self.synced_cloud_pub = self.create_publisher(
            PointCloud2, self.synced_cloud_topic, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.pending_clouds = deque()
        self.retry_timer = self.create_timer(
            self.retry_period_sec, self.process_pending_clouds)
        self.get_logger().info(
            f"Synchronizing {self.cloud_topic} with TF "
            f"{self.target_frame}->{self.source_frame}; publishing "
            f"{self.output_topic} and {self.synced_cloud_topic}"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        if len(self.pending_clouds) >= self.max_queue_size:
            dropped = self.pending_clouds.popleft()
            self._warn_drop(dropped.message, "queue full")
        self.pending_clouds.append(PendingCloud(msg, time.monotonic()))

    def process_pending_clouds(self) -> None:
        now = time.monotonic()
        while self.pending_clouds:
            pending = self.pending_clouds[0]
            if now - pending.received_at > self.max_tf_wait_sec:
                self.pending_clouds.popleft()
                self._warn_drop(pending.message, "TF wait timeout")
                continue

            msg = pending.message
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    self._stamp_to_time(msg.header.stamp),
                    timeout=Duration(seconds=0.0),
                )
            except Exception:
                return

            self.pending_clouds.popleft()
            self._publish_synced_pair(msg, transform)

    def _publish_synced_pair(self, msg: PointCloud2, transform) -> None:
        cloud_stamp = Header()
        cloud_stamp.stamp = msg.header.stamp
        cloud_stamp.frame_id = msg.header.frame_id

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.target_frame
        odom.child_frame_id = self.source_frame
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(odom)
        self.cloud_stamp_pub.publish(cloud_stamp)
        self.synced_cloud_pub.publish(msg)

    def _warn_drop(self, msg: PointCloud2, reason: str) -> None:
        self.get_logger().warn(
            f"Dropping cloud at "
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}: {reason}",
            throttle_duration_sec=2.0,
        )

    @staticmethod
    def _stamp_to_time(stamp: TimeMsg) -> Time:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return Time()
        return Time.from_msg(stamp)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
