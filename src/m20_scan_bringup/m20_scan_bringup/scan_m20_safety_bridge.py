#!/usr/bin/env python3

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool

from .safety_bridge_core import (
    BridgeState,
    Command,
    FreshnessLimits,
    InputState,
    NavCommandLimits,
    NavCommandShaper,
    RecentStampPairTracker,
    SafetyStateMachine,
    health_reasons,
)


RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

RELIABLE_SAFETY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

NAVIGATION_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ScanM20SafetyBridge(Node):
    REQUIRED_MOTION_STATE = 17
    REQUIRED_GAIT = 0x1001

    def __init__(self):
        super().__init__("scan_m20_safety_bridge")

        self.enable_m20_output = bool(
            self.declare_parameter("enable_m20_output", False).value)
        self.auto_arm_enabled = bool(
            self.declare_parameter("auto_arm_enabled", False).value)
        self.auto_arm_stable_sec = max(
            0.0,
            float(self.declare_parameter(
                "auto_arm_stable_sec", 2.0).value),
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 20.0).value)
        self.stop_cycles = int(
            self.declare_parameter("stop_cycles", 20).value)
        self.nav_frame_id = int(
            self.declare_parameter("nav_frame_id", 0).value)
        self.cloud_stamp_history_sec = float(
            self.declare_parameter(
                "cloud_stamp_history_sec", 1.0).value)

        self.nav_limits = NavCommandLimits(
            forward_speed=float(
                self.declare_parameter("forward_speed", 0.15).value),
            yaw_speed=float(
                self.declare_parameter("yaw_speed", 0.35).value),
            yaw_zero_epsilon=float(
                self.declare_parameter("yaw_zero_epsilon", 0.04).value),
        )
        self.freshness_limits = FreshnessLimits(
            command=float(
                self.declare_parameter("command_timeout", 0.20).value),
            body_pose=float(
                self.declare_parameter("body_pose_timeout", 0.50).value),
            sensor_pose=float(
                self.declare_parameter("sensor_pose_timeout", 0.80).value),
            cloud=float(
                self.declare_parameter("front_cloud_timeout", 0.80).value),
            motion_info=float(
                self.declare_parameter("motion_info_timeout", 0.50).value),
            max_sensor_cloud_stamp_delta=float(
                self.declare_parameter(
                    "max_sensor_cloud_stamp_delta", 0.02).value),
        )

        self.command_topic = self.declare_parameter(
            "command_topic", "/scan/cmd_vel_debug").value
        self.body_pose_topic = self.declare_parameter(
            "body_pose_topic", "/lio/robo/odom").value
        self.sensor_pose_topic = self.declare_parameter(
            "sensor_pose_topic", "/scan/sensor_pose").value
        self.front_cloud_stamp_topic = self.declare_parameter(
            "front_cloud_stamp_topic", "/scan/front_cloud_stamp").value
        self.motion_info_topic = self.declare_parameter(
            "motion_info_topic", "/MOTION_INFO").value
        self.preview_topic = self.declare_parameter(
            "preview_topic", "/scan/nav_cmd_preview").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/scan/nav_safety_status").value
        self.nav_cmd_topic = self.declare_parameter(
            "nav_cmd_topic", "/NAV_CMD").value
        self.navigation_enabled_topic = self.declare_parameter(
            "navigation_enabled_topic", "/scan/navigation_enabled").value
        self.arm_service_name = self.declare_parameter(
            "arm_service", "/scan/arm_nav_cmd").value

        self.inputs = InputState()
        self.sensor_cloud_pairs = RecentStampPairTracker(
            retention_sec=self.cloud_stamp_history_sec,
            tolerance_sec=self.freshness_limits.max_sensor_cloud_stamp_delta,
        )
        self.last_command = Command()
        self.preview_command = Command()
        self.shaper = NavCommandShaper(self.nav_limits)
        self.state_machine = SafetyStateMachine()
        self.motion_state = None
        self.gait_state = None
        self.motion_info_finite = True
        self.stop_reason = ""
        self.nav_pub = None
        self.nav_cmd_type = None
        self.motion_info_sub = None
        self.drdds_error = ""
        self.navigation_enabled = False
        self.manual_disarm_latched = False
        self.auto_arm_ready_since = None

        self.preview_pub = self.create_publisher(
            Twist, self.preview_topic, 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, self.status_topic, 10)
        self.navigation_enabled_pub = self.create_publisher(
            Bool, self.navigation_enabled_topic, NAVIGATION_STATE_QOS)
        self.command_sub = self.create_subscription(
            Twist,
            self.command_topic,
            self._command_callback,
            RELIABLE_SAFETY_QOS,
        )
        self.body_pose_sub = self.create_subscription(
            Odometry,
            self.body_pose_topic,
            self._body_pose_callback,
            qos_profile_sensor_data,
        )
        self.sensor_pose_sub = self.create_subscription(
            Odometry,
            self.sensor_pose_topic,
            self._sensor_pose_callback,
            qos_profile_sensor_data,
        )
        self.front_cloud_sub = self.create_subscription(
            Header,
            self.front_cloud_stamp_topic,
            self._front_cloud_callback,
            RELIABLE_SAFETY_QOS,
        )
        self.arm_service = self.create_service(
            SetBool, self.arm_service_name, self._handle_arm)
        self._publish_navigation_enabled(False)

        if self.enable_m20_output:
            self._initialize_drdds_runtime()

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 0.1), self._tick)
        mode = "REAL-CAPABLE DISARMED" if self.enable_m20_output else "DRY-RUN"
        self.get_logger().info(
            f"M20 NAV_CMD safety bridge started in {mode}; "
            f"preview={self.preview_topic}; "
            f"auto_arm_enabled={self.auto_arm_enabled}")

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
            MotionInfo,
            self.motion_info_topic,
            self._motion_info_callback,
            RELIABLE_QOS,
        )
        self.drdds_error = ""
        return True

    def _command_callback(self, msg: Twist) -> None:
        self.inputs.command_rx = time.monotonic()
        self.last_command = Command(
            vx=msg.linear.x,
            vy=msg.linear.y,
            wz=msg.angular.z,
        )

    def _body_pose_callback(self, _msg: Odometry) -> None:
        self.inputs.body_pose_rx = time.monotonic()

    def _sensor_pose_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        stamp_ns = self._stamp_ns(msg.header.stamp)
        self.inputs.sensor_pose_rx = now
        self.inputs.sensor_pose_stamp_ns = stamp_ns
        if self.sensor_cloud_pairs.add_pose(now, stamp_ns):
            self.inputs.sensor_cloud_pair_rx = (
                self.sensor_cloud_pairs.last_match_rx)

    def _front_cloud_callback(self, msg: Header) -> None:
        now = time.monotonic()
        stamp_ns = self._stamp_ns(msg.stamp)
        self.inputs.cloud_rx = now
        self.inputs.cloud_stamp_ns = stamp_ns
        if self.sensor_cloud_pairs.add_cloud(now, stamp_ns):
            self.inputs.sensor_cloud_pair_rx = (
                self.sensor_cloud_pairs.last_match_rx)

    def _motion_info_callback(self, msg) -> None:
        self.inputs.motion_info_rx = time.monotonic()
        try:
            data = msg.data
            self.motion_state = int(data.motion_state.state)
            self.gait_state = int(data.gait_state.gait)
            self.motion_info_finite = all(math.isfinite(value) for value in (
                data.vel_x,
                data.vel_y,
                data.vel_yaw,
            ))
        except (AttributeError, TypeError, ValueError):
            self.motion_state = None
            self.gait_state = None
            self.motion_info_finite = False

    def _handle_arm(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self.manual_disarm_latched = True
            self.auto_arm_ready_since = None
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

        if not self._arm_bridge("manual"):
            response.success = False
            response.message = "bridge_not_disarmed"
            return response

        self.manual_disarm_latched = False
        response.success = True
        response.message = "armed"
        return response

    def _arm_blockers(self, now=None) -> list[str]:
        now = time.monotonic() if now is None else now
        blockers = []
        if not self.enable_m20_output:
            blockers.append("enable_m20_output_false")
        elif not self._initialize_drdds_runtime():
            blockers.append("drdds_unavailable")
        if self.state_machine.state is not BridgeState.DISARMED:
            blockers.append("bridge_not_disarmed")
        blockers.extend(self._health_reasons(
            require_motion_info=True, now=now))
        blockers.extend(self._robot_state_reasons())
        if self.count_publishers(self.nav_cmd_topic) != 0:
            blockers.append("nav_cmd_publisher_exists")
        if self._command_is_nonzero(self.last_command):
            blockers.append("command_not_zero")
        return blockers

    def _robot_state_reasons(self) -> list[str]:
        if not self.motion_info_finite:
            return ["motion_info_nonfinite"]
        reasons = []
        if self.motion_state != self.REQUIRED_MOTION_STATE:
            reasons.append("motion_state_not_rl")
        if self.gait_state != self.REQUIRED_GAIT:
            reasons.append("gait_not_basic")
        return reasons

    def _arm_bridge(self, source: str) -> bool:
        if self.state_machine.state is not BridgeState.DISARMED:
            return False
        self.nav_pub = self.create_publisher(
            self.nav_cmd_type, self.nav_cmd_topic, RELIABLE_QOS)
        if not self.state_machine.arm():
            self.destroy_publisher(self.nav_pub)
            self.nav_pub = None
            return False
        self.stop_reason = ""
        self.auto_arm_ready_since = None
        self._publish_navigation_enabled(True)
        self.get_logger().warn(
            f"M20 /NAV_CMD output ARMED ({source})")
        return True

    def _update_auto_arm(self, now: float, blockers: list[str]) -> bool:
        eligible = (
            self.auto_arm_enabled
            and self.enable_m20_output
            and not self.manual_disarm_latched
            and self.state_machine.state is BridgeState.DISARMED
        )
        if not eligible:
            self.auto_arm_ready_since = None
            return False
        if blockers:
            self.auto_arm_ready_since = None
            return False
        if self.auto_arm_ready_since is None:
            self.auto_arm_ready_since = now
        if now - self.auto_arm_ready_since < self.auto_arm_stable_sec:
            return False
        return self._arm_bridge("automatic")

    def _tick(self) -> None:
        now = time.monotonic()
        base_reasons = self._health_reasons(
            require_motion_info=False, now=now)

        if base_reasons:
            self.shaper.reset()
            self.preview_command = Command()
        else:
            self.preview_command = self.shaper.map(
                self.last_command.vx,
                self.last_command.vy,
                self.last_command.wz,
            )
        self._publish_preview(self.preview_command)

        if self.state_machine.state is BridgeState.ARMED:
            armed_reasons = self._health_reasons(
                require_motion_info=True, now=now)
            armed_reasons.extend(self._robot_state_reasons())
            if self.count_publishers(self.nav_cmd_topic) > 1:
                armed_reasons.append("additional_nav_cmd_publisher")
            if armed_reasons:
                self._begin_stop(",".join(armed_reasons))
            else:
                if not self._publish_nav_command(self.preview_command):
                    self._abort_output("nav_cmd_publish_failed")

        if self.state_machine.state is BridgeState.STOPPING:
            if not self._publish_nav_command(Command()):
                self._abort_output("nav_cmd_zero_publish_failed")
            elif self.state_machine.record_stop_publish():
                self._destroy_nav_publisher()

        auto_arm_blockers = []
        if (
            self.auto_arm_enabled
            and self.enable_m20_output
            and not self.manual_disarm_latched
            and self.state_machine.state is BridgeState.DISARMED
        ):
            auto_arm_blockers = self._arm_blockers(now)
        self._update_auto_arm(now, auto_arm_blockers)

        status_reasons = list(base_reasons)
        if self.enable_m20_output:
            status_reasons.extend(self._health_reasons(
                require_motion_info=True, now=now))
            status_reasons.extend(self._robot_state_reasons())
        else:
            status_reasons.append("enable_m20_output_false")
        self._publish_status(list(dict.fromkeys(status_reasons)))

    def _health_reasons(self, require_motion_info: bool, now=None) -> list[str]:
        return health_reasons(
            time.monotonic() if now is None else now,
            self.inputs,
            self.freshness_limits,
            require_motion_info,
            use_completed_pair=True,
        )

    def _begin_stop(self, reason: str) -> None:
        self.auto_arm_ready_since = None
        self._publish_navigation_enabled(False)
        if self.state_machine.state is BridgeState.DISARMED:
            return
        if self.state_machine.state is not BridgeState.STOPPING:
            self.stop_reason = reason
            self.get_logger().error(f"Disarming M20 output: {reason}")
            self.shaper.reset()
            self.preview_command = Command()
        self.state_machine.begin_stop(self.stop_cycles)

    def _publish_navigation_enabled(self, enabled: bool) -> None:
        self.navigation_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.navigation_enabled_pub.publish(msg)

    def _publish_preview(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x = float(command.vx)
        msg.linear.y = 0.0
        msg.angular.z = float(command.wz)
        self.preview_pub.publish(msg)

    def _publish_nav_command(self, command: Command) -> bool:
        if self.nav_pub is None or self.nav_cmd_type is None:
            return False
        try:
            values = (command.vx, command.vy, command.wz)
            if not all(math.isfinite(value) for value in values):
                command = Command()
            msg = self.nav_cmd_type()
            msg.header.frame_id = self.nav_frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data.x_vel = float(command.vx)
            msg.data.y_vel = 0.0
            msg.data.yaw_vel = float(command.wz)
            self.nav_pub.publish(msg)
        except Exception as exc:
            self.drdds_error = str(exc)
            self.get_logger().error(f"Failed to publish /NAV_CMD: {exc}")
            return False
        return True

    def _abort_output(self, reason: str) -> None:
        self.stop_reason = reason
        self.manual_disarm_latched = True
        self.shaper.reset()
        self.preview_command = Command()
        self.state_machine.abort_output()
        self._destroy_nav_publisher()

    def _destroy_nav_publisher(self) -> None:
        if self.nav_pub is not None:
            self.destroy_publisher(self.nav_pub)
            self.nav_pub = None
        self._publish_navigation_enabled(False)
        self.get_logger().warn(
            f"M20 output DISARMED: {self.stop_reason}")

    def _publish_status(self, reasons: list[str]) -> None:
        status = DiagnosticStatus()
        status.name = "scan_m20_safety_bridge"
        status.hardware_id = "m20"
        status.level = DiagnosticStatus.OK if not reasons else DiagnosticStatus.WARN
        status.message = "ready" if not reasons else ",".join(reasons)
        status.values = [
            KeyValue(key="state", value=self.state_machine.state.value),
            KeyValue(
                key="enable_m20_output",
                value=str(self.enable_m20_output).lower()),
            KeyValue(
                key="auto_arm_enabled",
                value=str(self.auto_arm_enabled).lower()),
            KeyValue(
                key="motion_state",
                value=str(self.motion_state)),
            KeyValue(
                key="gait_state",
                value=str(self.gait_state)),
            KeyValue(
                key="motion_info_finite",
                value=str(self.motion_info_finite).lower()),
            KeyValue(
                key="navigation_enabled",
                value=str(self.navigation_enabled).lower()),
            KeyValue(
                key="nav_publisher_created",
                value=str(self.nav_pub is not None).lower()),
            KeyValue(key="stop_reason", value=self.stop_reason),
            KeyValue(key="drdds_error", value=self.drdds_error),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)

    def _command_is_nonzero(self, command: Command) -> bool:
        return (
            command.vx > 1e-6
            or abs(command.wz) > self.nav_limits.yaw_zero_epsilon
        )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def destroy_node(self):
        if self.nav_pub is not None:
            self._publish_navigation_enabled(False)
            for _ in range(max(1, self.stop_cycles)):
                self._publish_nav_command(Command())
                time.sleep(1.0 / max(self.publish_rate, 0.1))
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
