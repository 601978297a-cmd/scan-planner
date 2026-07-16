from std_srvs.srv import SetBool

import rclpy

from m20_scan_bringup.scan_m20_safety_bridge import ScanM20SafetyBridge


def test_default_node_rejects_arm_without_creating_nav_publisher():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.nav_pub is None
        request = SetBool.Request()
        request.data = True
        response = node._handle_arm(request, SetBool.Response())
        assert not response.success
        assert "enable_m20_output_false" in response.message
        assert node.nav_pub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
