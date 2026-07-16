#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudFilterAdapter(Node):
    def __init__(self):
        super().__init__("cloud_filter_adapter")
        self.input_topic = self.declare_parameter("input_topic", "/lightning/current_scan").value
        self.output_topic = self.declare_parameter("output_topic", "/scan/current_scan_obstacles").value
        self.ground_z = float(self.declare_parameter("ground_z", 0.0).value)
        self.min_obstacle_height = float(self.declare_parameter("min_obstacle_height", 0.12).value)
        self.max_obstacle_height = float(self.declare_parameter("max_obstacle_height", 1.20).value)
        self.log_every_n = int(self.declare_parameter("log_every_n", 30).value)
        self.frame_id = self.declare_parameter("frame_id", "map").value

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(
            f"Filtering {self.input_topic} -> {self.output_topic}; "
            f"keeping z in [{self.ground_z + self.min_obstacle_height:.3f}, "
            f"{self.ground_z + self.max_obstacle_height:.3f}] in {self.frame_id}"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        field_names = [field.name for field in msg.fields]
        try:
            z_index = field_names.index("z")
        except ValueError:
            self.get_logger().warn("Input cloud has no z field; dropping frame", throttle_duration_sec=2.0)
            return

        min_z = self.ground_z + self.min_obstacle_height
        max_z = self.ground_z + self.max_obstacle_height
        kept = []
        total = 0
        for point in point_cloud2.read_points(msg, field_names=field_names, skip_nans=True):
            total += 1
            z = float(point[z_index])
            if min_z <= z <= max_z:
                kept.append(tuple(point))

        out = point_cloud2.create_cloud(msg.header, msg.fields, kept)
        out.header.frame_id = self.frame_id or msg.header.frame_id
        out.is_dense = False
        self.pub.publish(out)

        self.count += 1
        if self.log_every_n > 0 and self.count % self.log_every_n == 0:
            ratio = (len(kept) / total) if total else 0.0
            self.get_logger().info(f"filtered cloud: kept {len(kept)}/{total} points ({ratio:.1%})")


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
