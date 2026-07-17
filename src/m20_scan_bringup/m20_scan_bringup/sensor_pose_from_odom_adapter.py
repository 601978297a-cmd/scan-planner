#!/usr/bin/env python3

from collections import deque
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros


class SensorPoseFromOdomAdapter(Node):
    def __init__(self):
        super().__init__("sensor_pose_from_odom_adapter")
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.source_frame = self.declare_parameter("source_frame", "rslidar_front").value
        self.cloud_topic = self.declare_parameter("cloud_topic", "/rslidar_points").value
        self.body_pose_topic = self.declare_parameter(
            "body_pose_topic", "/lightning/odom").value
        self.output_topic = self.declare_parameter("output_topic", "/scan/sensor_pose").value
        self.cloud_stamp_topic = self.declare_parameter(
            "cloud_stamp_topic", "/scan/front_cloud_stamp").value
        self.lookup_timeout_sec = float(self.declare_parameter("lookup_timeout_sec", 0.1).value)
        self.max_body_pose_delta_sec = float(
            self.declare_parameter("max_body_pose_delta_sec", 0.5).value)

        self.body_poses = deque(maxlen=200)
        self.extrinsic = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pose_pub = self.create_publisher(Odometry, self.output_topic, qos_profile_sensor_data)
        self.cloud_stamp_pub = self.create_publisher(
            Header, self.cloud_stamp_topic, qos_profile_sensor_data)
        self.body_pose_sub = self.create_subscription(
            Odometry,
            self.body_pose_topic,
            self.body_pose_callback,
            qos_profile_sensor_data,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Publishing {self.output_topic} from {self.body_pose_topic} and TF "
            f"{self.base_frame}->{self.source_frame} on {self.cloud_topic} timestamps; "
            f"cloud stamp={self.cloud_stamp_topic}"
        )

    def body_pose_callback(self, msg: Odometry) -> None:
        self.body_poses.append(msg)

    def cloud_callback(self, cloud: PointCloud2) -> None:
        cloud_stamp = Header()
        cloud_stamp.stamp = cloud.header.stamp
        cloud_stamp.frame_id = cloud.header.frame_id
        self.cloud_stamp_pub.publish(cloud_stamp)

        if not self.body_poses:
            self.get_logger().warn(
                f"No body pose received from {self.body_pose_topic}",
                throttle_duration_sec=2.0,
            )
            return

        cloud_ns = self._stamp_nanoseconds(cloud.header.stamp)
        body_pose = min(
            self.body_poses,
            key=lambda pose: abs(self._stamp_nanoseconds(pose.header.stamp) - cloud_ns),
        )
        pose_ns = self._stamp_nanoseconds(body_pose.header.stamp)
        delta_sec = abs(pose_ns - cloud_ns) / 1e9
        if delta_sec > self.max_body_pose_delta_sec:
            self.get_logger().warn(
                f"Nearest body pose is {delta_sec:.3f}s from cloud timestamp; not publishing",
                throttle_duration_sec=2.0,
            )
            return
        if body_pose.header.frame_id and body_pose.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Body pose frame is {body_pose.header.frame_id}, expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return

        if self.extrinsic is None:
            try:
                self.extrinsic = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.source_frame,
                    Time(),
                    timeout=Duration(seconds=self.lookup_timeout_sec),
                )
            except Exception as exc:  # tf2_ros exception classes vary across distros.
                self.get_logger().warn(
                    f"TF lookup failed for {self.base_frame}->{self.source_frame}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return

        try:
            body_orientation = self._normalize_quaternion(body_pose.pose.pose.orientation)
            extrinsic_orientation = self._normalize_quaternion(
                self.extrinsic.transform.rotation)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        extrinsic_translation = self.extrinsic.transform.translation
        rotated_translation = self._rotate_vector(
            body_orientation,
            (extrinsic_translation.x, extrinsic_translation.y, extrinsic_translation.z),
        )
        sensor_orientation = self._multiply_quaternions(
            body_orientation,
            extrinsic_orientation,
        )
        body_position = body_pose.pose.pose.position

        sensor_pose = Odometry()
        sensor_pose.header.stamp = cloud.header.stamp
        sensor_pose.header.frame_id = self.target_frame
        sensor_pose.child_frame_id = self.source_frame
        sensor_pose.pose.pose.position.x = body_position.x + rotated_translation[0]
        sensor_pose.pose.pose.position.y = body_position.y + rotated_translation[1]
        sensor_pose.pose.pose.position.z = body_position.z + rotated_translation[2]
        sensor_pose.pose.pose.orientation.x = sensor_orientation[0]
        sensor_pose.pose.pose.orientation.y = sensor_orientation[1]
        sensor_pose.pose.pose.orientation.z = sensor_orientation[2]
        sensor_pose.pose.pose.orientation.w = sensor_orientation[3]
        self.pose_pub.publish(sensor_pose)

    @staticmethod
    def _stamp_nanoseconds(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 1e-9:
            raise ValueError("Quaternion has zero norm")
        return (
            quaternion.x / norm,
            quaternion.y / norm,
            quaternion.z / norm,
            quaternion.w / norm,
        )

    @staticmethod
    def _multiply_quaternions(lhs, rhs):
        lx, ly, lz, lw = lhs
        rx, ry, rz, rw = rhs
        result = (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
        norm = math.sqrt(sum(component * component for component in result))
        return tuple(component / norm for component in result)

    @staticmethod
    def _rotate_vector(quaternion, vector):
        qx, qy, qz, qw = quaternion
        vx, vy, vz = vector
        dot_uv = qx * vx + qy * vy + qz * vz
        dot_uu = qx * qx + qy * qy + qz * qz
        cross_x = qy * vz - qz * vy
        cross_y = qz * vx - qx * vz
        cross_z = qx * vy - qy * vx
        scale = qw * qw - dot_uu
        return (
            2.0 * dot_uv * qx + scale * vx + 2.0 * qw * cross_x,
            2.0 * dot_uv * qy + scale * vy + 2.0 * qw * cross_y,
            2.0 * dot_uv * qz + scale * vz + 2.0 * qw * cross_z,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseFromOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
