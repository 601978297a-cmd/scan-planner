#!/usr/bin/env python3

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from .safety_bridge_core import (
    BridgeState,
    Command,
    CommandLimits,
    FreshnessLimits,
    InputState,
    SafetyStateMachine,
    health_reasons,
    limit_command,
    slew_command,
)


RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ScanM20SafetyBridge(Node):
    def __init__(self):
        super().__init__("scan_m20_safety_bridge")

        self.enable_m20_output = bool(self.declare_parameter("enable_m20_output", False).value)
        self.yaw_deadzone_calibrated = bool(
            self.declare_parameter("yaw_deadzone_calibrated", False).value)
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.stop_cycles = int(self.declare_parameter("stop_cycles", 5).value)
        self.nav_frame_id = int(self.declare_parameter("nav_frame_id", 0).value)

        self.command_limits = CommandLimits(
            max_vx=float(self.declare_parameter("max_vx", 0.05).value),
            max_wz=float(self.declare_parameter("max_wz", 0.30).value),
            max_ax=float(self.declare_parameter("max_ax", 0.10).value),
            max_awz=float(self.declare_parameter("max_awz", 0.50).value),
            yaw_zero_epsilon=float(
                self.declare_parameter("yaw_zero_epsilon", 0.01).value),
            min_yaw_cmd=float(self.declare_parameter("min_yaw_cmd", 0.0).value),
        )
        self.freshness_limits = FreshnessLimits(
            command=float(self.declare_parameter("command_timeout", 0.20).value),
            body_pose=float(self.declare_parameter("body_pose_timeout", 0.50).value),
            sensor_pose=float(self.declare_parameter("sensor_pose_timeout", 0.50).value),
            cloud=float(self.declare_parameter("front_cloud_timeout", 0.30).value),
            motion_info=float(self.declare_parameter("motion_info_timeout", 0.50).value),
            max_sensor_cloud_stamp_delta=float(
                self.declare_parameter("max_sensor_cloud_stamp_delta", 0.02).value),
        )

        self.command_topic = self.declare_parameter(
            "command_topic", "/scan/cmd_vel_debug").value
        self.body_pose_topic = self.declare_parameter(
            "body_pose_topic", "/lio/robo/odom").value
        self.sensor_pose_topic = self.declare_parameter(
            "sensor_pose_topic", "/scan/sensor_pose").value
        self.front_cloud_stamp_topic = self.declare_parameter(
            "front_cloud_stamp_topic", "/scan/front_cloud_stamp").value
        self.preview_topic = self.declare_parameter(
            "preview_topic", "/scan/nav_cmd_preview").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/scan/nav_safety_status").value
        self.nav_cmd_topic = self.declare_parameter("nav_cmd_topic", "/NAV_CMD").value

        self.inputs = InputState()
        self.last_command = Command()
        self.preview_command = Command()
        self.last_tick = time.monotonic()
        self.state_machine = SafetyStateMachine()
        self.stop_reason = ""
        self.nav_pub = None
        self.nav_cmd_type = None
        self.motion_info_sub = None
        self.drdds_error = ""

        self.preview_pub = self.create_publisher(Twist, self.preview_topic, 10)
        self.status_pub = self.create_publisher(DiagnosticArray, self.status_topic, 10)
        self.command_sub = self.create_subscription(
            Twist, self.command_topic, self._command_callback, 10)
        self.body_pose_sub = self.create_subscription(
            Odometry, self.body_pose_topic, self._body_pose_callback, qos_profile_sensor_data)
        self.sensor_pose_sub = self.create_subscription(
            Odometry, self.sensor_pose_topic, self._sensor_pose_callback, qos_profile_sensor_data)
        self.front_cloud_sub = self.create_subscription(
            Header,
            self.front_cloud_stamp_topic,
            self._front_cloud_callback,
            qos_profile_sensor_data,
        )
        self.arm_service = self.create_service(SetBool, "/scan/arm_nav_cmd", self._handle_arm)

        if self.enable_m20_output:
            self._initialize_drdds_runtime()

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        mode = "REAL-CAPABLE DISARMED" if self.enable_m20_output else "DRY-RUN"
        self.get_logger().info(
            f"M20 safety bridge started in {mode}; preview={self.preview_topic}; "
            f"nav_cmd_publisher_created={self.nav_pub is not None}")

    def _initialize_drdds_runtime(self) -> bool:
        if self.nav_cmd_type is not None:
            return True
        try:
            from drdds.msg import MotionInfo, NavCmd
        except Exception as exc:
            self.drdds_error = str(exc)
            self.get_logger().error(f"drdds runtime unavailable: {exc}")
            return False
        self.nav_cmd_type = NavCmd
        self.motion_info_sub = self.create_subscription(
            MotionInfo, "/MOTION_INFO", self._motion_info_callback, RELIABLE_QOS)
        return True

    def _command_callback(self, msg: Twist) -> None:
        self.inputs.command_rx = time.monotonic()
        self.last_command = Command(vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)

    def _body_pose_callback(self, _msg: Odometry) -> None:
        self.inputs.body_pose_rx = time.monotonic()

    def _sensor_pose_callback(self, msg: Odometry) -> None:
        self.inputs.sensor_pose_rx = time.monotonic()
        self.inputs.sensor_pose_stamp_ns = self._stamp_ns(msg.header.stamp)

    def _front_cloud_callback(self, msg: Header) -> None:
        self.inputs.cloud_rx = time.monotonic()
        self.inputs.cloud_stamp_ns = self._stamp_ns(msg.stamp)

    def _motion_info_callback(self, _msg) -> None:
        self.inputs.motion_info_rx = time.monotonic()

    def _handle_arm(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            if self.state_machine.state is BridgeState.DISARMED:
                response.success = True
                response.message = "already disarmed"
            else:
                self._begin_stop("manual_disarm")
                response.success = True
                response.message = "stopping with zero command sequence"
            return response

        reasons = self._arm_blockers()
        if reasons:
            response.success = False
            response.message = ",".join(reasons)
            return response

        self.nav_pub = self.create_publisher(self.nav_cmd_type, self.nav_cmd_topic, RELIABLE_QOS)
        if not self.state_machine.arm():
            self.destroy_publisher(self.nav_pub)
            self.nav_pub = None
            response.success = False
            response.message = "bridge_not_disarmed"
            return response

        self.stop_reason = ""
        response.success = True
        response.message = "armed"
        self.get_logger().warn("M20 /NAV_CMD output ARMED")
        return response

    def _arm_blockers(self) -> list[str]:
        blockers = []
        if not self.enable_m20_output:
            blockers.append("enable_m20_output_false")
        if not self.yaw_deadzone_calibrated:
            blockers.append("yaw_deadzone_not_calibrated")
        if self.state_machine.state is not BridgeState.DISARMED:
            blockers.append("bridge_not_disarmed")
        if self.enable_m20_output and not self._initialize_drdds_runtime():
            blockers.append("drdds_unavailable")
        blockers.extend(self._health_reasons(require_motion_info=True))
        if self.count_publishers(self.nav_cmd_topic) != 0:
            blockers.append("nav_cmd_publisher_exists")
        return blockers

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.25)
        self.last_tick = now
        base_reasons = self._health_reasons(require_motion_info=False, now=now)

        if base_reasons:
            self.preview_command = Command()
        else:
            target = limit_command(
                self.last_command.vx, self.last_command.wz, self.command_limits)
            self.preview_command = slew_command(
                self.preview_command, target, dt, self.command_limits)
        self._publish_preview(self.preview_command)

        if self.state_machine.state is BridgeState.ARMED:
            armed_reasons = self._health_reasons(require_motion_info=True, now=now)
            if self.count_publishers(self.nav_cmd_topic) > 1:
                armed_reasons.append("additional_nav_cmd_publisher")
            if armed_reasons:
                self._begin_stop(",".join(armed_reasons))
            else:
                self._publish_nav_command(self.preview_command)

        if self.state_machine.state is BridgeState.STOPPING:
            self._publish_nav_command(Command())
            if self.state_machine.record_stop_publish():
                self._destroy_nav_publisher()

        self._publish_status(base_reasons)

    def _health_reasons(self, require_motion_info: bool, now=None) -> list[str]:
        return health_reasons(
            time.monotonic() if now is None else now,
            self.inputs,
            self.freshness_limits,
            require_motion_info,
        )

    def _begin_stop(self, reason: str) -> None:
        if self.state_machine.state is BridgeState.DISARMED:
            return
        if self.state_machine.state is not BridgeState.STOPPING:
            self.stop_reason = reason
            self.get_logger().error(f"Disarming M20 output: {reason}")
        self.state_machine.begin_stop(self.stop_cycles)

    def _publish_preview(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x = float(command.vx)
        msg.linear.y = 0.0
        msg.angular.z = float(command.wz)
        self.preview_pub.publish(msg)

    def _publish_nav_command(self, command: Command) -> None:
        if self.nav_pub is None or self.nav_cmd_type is None:
            return
        msg = self.nav_cmd_type()
        msg.header.frame_id = self.nav_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data.x_vel = float(command.vx)
        msg.data.y_vel = 0.0
        msg.data.yaw_vel = float(command.wz)
        self.nav_pub.publish(msg)

    def _destroy_nav_publisher(self) -> None:
        if self.nav_pub is not None:
            self.destroy_publisher(self.nav_pub)
            self.nav_pub = None
        self.get_logger().warn(f"M20 output DISARMED: {self.stop_reason}")

    def _publish_status(self, base_reasons: list[str]) -> None:
        status = DiagnosticStatus()
        status.name = "scan_m20_safety_bridge"
        status.hardware_id = "m20"
        status.level = DiagnosticStatus.OK if not base_reasons else DiagnosticStatus.WARN
        status.message = "ready" if not base_reasons else ",".join(base_reasons)
        status.values = [
            KeyValue(key="state", value=self.state_machine.state.value),
            KeyValue(key="enable_m20_output", value=str(self.enable_m20_output).lower()),
            KeyValue(
                key="yaw_deadzone_calibrated",
                value=str(self.yaw_deadzone_calibrated).lower()),
            KeyValue(key="nav_publisher_created", value=str(self.nav_pub is not None).lower()),
            KeyValue(key="stop_reason", value=self.stop_reason),
            KeyValue(key="drdds_error", value=self.drdds_error),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def destroy_node(self):
        if self.nav_pub is not None:
            for _ in range(max(1, self.stop_cycles)):
                self._publish_nav_command(Command())
                time.sleep(1.0 / self.publish_rate)
            self._destroy_nav_publisher()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanM20SafetyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
