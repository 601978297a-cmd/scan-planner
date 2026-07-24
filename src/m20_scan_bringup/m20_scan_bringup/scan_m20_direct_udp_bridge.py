import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .m20_udp_protocol import (
    M20UdpLink,
    UdpAxis,
    scale_direct_udp_axis,
)


class ScanM20DirectUdpBridge(Node):
    def __init__(self):
        super().__init__("scan_m20_direct_udp_bridge")

        self.command_topic = self.declare_parameter(
            "command_topic", "/scan/cmd_vel_debug").value
        self.udp_target_host = self.declare_parameter(
            "udp_target_host", "10.21.31.103").value
        self.udp_target_port = int(
            self.declare_parameter("udp_target_port", 30000).value)
        self.scale_x = float(
            self.declare_parameter("scale_x", 3.0).value)
        self.scale_y = float(
            self.declare_parameter("scale_y", 3.0).value)
        self.scale_yaw = float(
            self.declare_parameter("scale_yaw", 2.0).value)
        self.timeout_sec = max(
            0.001,
            float(self.declare_parameter("timeout_ms", 500.0).value)
            / 1000.0,
        )
        self.send_stand_on_start = bool(
            self.declare_parameter("send_stand_on_start", True).value)

        self.udp_link = M20UdpLink(
            self.udp_target_host,
            self.udp_target_port,
        )
        self.last_command_time = None
        self.timeout_stop_sent = True
        self.closed = False

        self.command_subscription = self.create_subscription(
            Twist,
            self.command_topic,
            self._command_callback,
            10,
        )
        self.timeout_timer = self.create_timer(
            max(0.01, self.timeout_sec / 2.0),
            self._timeout_callback,
        )

        if self.send_stand_on_start:
            self.udp_link.send_motion_state(1)
            self.get_logger().info(
                "Sent stand command (MotionParam=1) to M20")

        self.get_logger().info(
            "Direct M20 UDP bridge started: "
            f"topic={self.command_topic}, "
            f"target={self.udp_target_host}:{self.udp_target_port}, "
            f"scales=({self.scale_x:.2f}, {self.scale_y:.2f}, "
            f"{self.scale_yaw:.2f}), timeout={self.timeout_sec:.3f}s")

    def _command_callback(self, message: Twist) -> None:
        axis = scale_direct_udp_axis(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self.scale_x,
            self.scale_y,
            self.scale_yaw,
        )
        try:
            self.udp_link.send_axis(axis)
        except OSError as exc:
            self.get_logger().error(f"Failed to send UDP command: {exc}")
            return
        self.last_command_time = time.monotonic()
        self.timeout_stop_sent = False

    def _timeout_callback(self) -> None:
        if self.last_command_time is None or self.timeout_stop_sent:
            return
        if time.monotonic() - self.last_command_time <= self.timeout_sec:
            return
        try:
            self.udp_link.send_axis(UdpAxis())
        except OSError as exc:
            self.get_logger().error(f"Failed to send UDP timeout stop: {exc}")
            return
        self.timeout_stop_sent = True
        self.get_logger().info("Velocity timeout; sent zero UDP command")

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.udp_link.send_axis(UdpAxis())
        except OSError as exc:
            self.get_logger().error(f"Failed to send UDP shutdown stop: {exc}")
        self.udp_link.close()
        self.closed = True

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanM20DirectUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

