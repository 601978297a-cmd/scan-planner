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
