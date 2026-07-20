from std_srvs.srv import SetBool

import rclpy
from std_msgs.msg import Header

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


def test_node_uses_lightweight_front_cloud_stamp():
    rclpy.init()
    node = ScanM20SafetyBridge()
    try:
        assert node.front_cloud_stamp_topic == "/scan/front_cloud_stamp"
        assert node.front_cloud_sub.topic_name == "/scan/front_cloud_stamp"
        assert node.front_cloud_sub.msg_type is Header

        stamp = Header()
        stamp.stamp.sec = 12
        stamp.stamp.nanosec = 345
        node._front_cloud_callback(stamp)

        assert node.inputs.cloud_rx is not None
        assert node.inputs.cloud_stamp_ns == 12_000_000_345
    finally:
        node.destroy_node()
        rclpy.shutdown()
