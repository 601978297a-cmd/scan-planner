import time

from geometry_msgs.msg import Twist
import rclpy

import m20_scan_bringup.scan_m20_direct_udp_bridge as direct_bridge
from m20_scan_bringup.m20_udp_protocol import UdpAxis


class CapturingUdpLink:
    def __init__(self):
        self.axes = []
        self.motion_states = []
        self.closed = False

    def send_axis(self, axis):
        self.axes.append(axis)

    def send_motion_state(self, state):
        self.motion_states.append(state)

    def close(self):
        self.closed = True


def make_node(monkeypatch):
    link = CapturingUdpLink()
    monkeypatch.setattr(
        direct_bridge,
        "M20UdpLink",
        lambda _host, _port: link,
    )
    node = direct_bridge.ScanM20DirectUdpBridge()
    return node, link


def test_direct_bridge_stays_silent_until_nonzero_twist(monkeypatch):
    rclpy.init()
    node, link = make_node(monkeypatch)
    try:
        assert link.motion_states == []

        node._command_callback(Twist())
        assert link.axes == []

        command = Twist()
        command.linear.x = 0.15
        command.linear.y = -0.20
        command.angular.z = 0.20
        node._command_callback(command)

        axis = link.axes[-1]
        assert abs(axis.x - 0.45) < 1e-9
        assert abs(axis.y + 0.60) < 1e-9
        assert abs(axis.yaw - 0.40) < 1e-9
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_direct_bridge_timeout_and_shutdown_send_zero(monkeypatch):
    rclpy.init()
    node, link = make_node(monkeypatch)
    try:
        command = Twist()
        command.linear.x = 0.1
        node._command_callback(command)
        node.last_command_time = time.monotonic() - 1.0
        node._timeout_callback()
        assert link.axes[-1] == UdpAxis()

        count_after_timeout = len(link.axes)
        node._timeout_callback()
        assert len(link.axes) == count_after_timeout

        node.destroy_node()
        assert link.axes[-1] == UdpAxis()
        assert link.closed
    finally:
        if not node.closed:
            node.destroy_node()
        rclpy.shutdown()


def test_direct_bridge_shutdown_before_navigation_stays_silent(monkeypatch):
    rclpy.init()
    node, link = make_node(monkeypatch)
    try:
        node._command_callback(Twist())
        node.destroy_node()
        assert link.axes == []
        assert link.motion_states == []
        assert link.closed
    finally:
        if not node.closed:
            node.destroy_node()
        rclpy.shutdown()
