import copy
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def sanitize_path(path, global_frame):
    if not path.poses:
        return None
    if path.header.frame_id not in ("", global_frame):
        return None

    poses = []
    for pose_stamped in path.poses:
        if pose_stamped.header.frame_id not in ("", global_frame):
            return None
        position = pose_stamped.pose.position
        if not all(math.isfinite(value) for value in (
                position.x, position.y, position.z)):
            return None
        pose = copy.deepcopy(pose_stamped)
        pose.header.frame_id = global_frame
        pose.pose.position.z = 0.0
        poses.append(pose)

    if len(poses) == 1:
        return None

    sanitized = Path()
    sanitized.header = copy.deepcopy(path.header)
    sanitized.header.frame_id = global_frame
    sanitized.poses = poses
    return sanitized


class SmacToScanBridge(Node):
    def __init__(self):
        super().__init__("smac_to_scan_bridge")

        self.global_frame = self.declare_parameter(
            "global_frame", "world").value
        self.goal_topic = self.declare_parameter(
            "goal_topic", "/move_base_simple/goal").value
        self.path_topic = self.declare_parameter(
            "path_topic", "/initial_path").value
        self.planner_action = self.declare_parameter(
            "planner_action", "/compute_path_to_pose").value
        self.planner_id = self.declare_parameter(
            "planner_id", "GridBased").value

        self.path_pub = self.create_publisher(Path, self.path_topic, 1)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.action_client = ActionClient(
            self, ComputePathToPose, self.planner_action)
        self.request_serial = 0

        self.get_logger().info(
            "Smac bridge ready: %s -> %s -> %s" % (
                self.goal_topic, self.planner_action, self.path_topic))

    def goal_callback(self, message):
        if message.header.frame_id != self.global_frame:
            self.get_logger().error(
                "Reject goal in frame '%s'; expected '%s'" % (
                    message.header.frame_id, self.global_frame))
            return

        position = message.pose.position
        if not all(math.isfinite(value) for value in (
                position.x, position.y, position.z)):
            self.get_logger().error("Reject goal with non-finite position")
            return

        if not self.action_client.server_is_ready():
            self.get_logger().error(
                "Smac planner action is not ready: %s" %
                self.planner_action)
            return

        self.request_serial += 1
        serial = self.request_serial

        goal = ComputePathToPose.Goal()
        goal.goal = copy.deepcopy(message)
        goal.goal.header.frame_id = self.global_frame
        goal.goal.pose.position.z = 0.0
        goal.planner_id = self.planner_id
        goal.use_start = False

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, request=serial:
            self.goal_response_callback(result, request))

    def goal_response_callback(self, future, serial):
        if serial != self.request_serial:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error("Smac goal request failed: %s" % exc)
            return
        if not goal_handle.accepted:
            self.get_logger().error("Smac rejected the planning request")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, request=serial:
            self.path_result_callback(result, request))

    def path_result_callback(self, future, serial):
        if serial != self.request_serial:
            return
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error("Smac planning failed: %s" % exc)
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "Smac planning ended with status %d" %
                wrapped_result.status)
            return

        raw_path = wrapped_result.result.path
        path = sanitize_path(raw_path, self.global_frame)
        if path is None:
            self.get_logger().error(
                "Reject invalid or empty path returned by Smac")
            return

        path.header.stamp = self.get_clock().now().to_msg()
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
        self.path_pub.publish(path)
        self.get_logger().info(
            "Published SCAN reference path with %d poses "
            "(Smac returned %d)" % (
                len(path.poses), len(raw_path.poses)))


def main(args=None):
    rclpy.init(args=args)
    node = SmacToScanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
