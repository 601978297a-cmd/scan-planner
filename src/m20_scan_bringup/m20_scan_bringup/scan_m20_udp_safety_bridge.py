#!/usr/bin/env python3

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool

from .m20_udp_protocol import (
    M20UdpLink,
    UdpAxis,
    UdpCommandMapper,
    UdpMappingLimits,
    find_conflicting_processes,
    slew_udp_axis,
)
from .safety_bridge_core import (
    BridgeState,
    Command,
    CommandLimits,
    FreshnessLimits,
    InputState,
    RecentStampPairTracker,
    SafetyStateMachine,
    health_reasons,
    limit_command,
    slew_command,
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


class ScanM20UdpSafetyBridge(Node):
    def __init__(self):
        super().__init__("scan_m20_udp_safety_bridge")

        self.enable_udp_output = bool(
            self.declare_parameter("enable_udp_output", False).value)
        self.timer_rate = float(self.declare_parameter("timer_rate", 20.0).value)
        self.command_rate = float(
            self.declare_parameter("command_rate", 20.0).value)
        self.preview_rate = float(
            self.declare_parameter("preview_rate", 20.0).value)
        self.status_rate = float(
            self.declare_parameter("status_rate", 2.0).value)
        self.heartbeat_rate = float(
            self.declare_parameter("heartbeat_rate", 1.0).value)
        self.heartbeat_timeout = float(
            self.declare_parameter("heartbeat_timeout", 2.0).value)
        self.stop_cycles = int(self.declare_parameter("stop_cycles", 20).value)
        self.auto_arm_enabled = bool(
            self.declare_parameter("auto_arm_enabled", False).value)
        self.auto_arm_stable_sec = max(
            0.0,
            float(self.declare_parameter(
                "auto_arm_stable_sec", 2.0).value),
        )
        self.cloud_stamp_history_sec = float(
            self.declare_parameter(
                "cloud_stamp_history_sec", 1.0).value)
        self.conflict_check_period = float(
            self.declare_parameter("conflict_check_period", 0.5).value)
        self.conflicting_process_names = tuple(
            self.declare_parameter(
                "conflicting_process_names",
                ["key_test", "m20_udp_drive_test.py"],
            ).value
        )

        self.udp_target_host = self.declare_parameter(
            "udp_target_host", "10.21.31.103").value
        self.udp_target_port = int(
            self.declare_parameter("udp_target_port", 30000).value)
        self.udp_bind_host = self.declare_parameter("udp_bind_host", "").value
        self.udp_bind_port = int(
            self.declare_parameter("udp_bind_port", 0).value)

        self.command_limits = CommandLimits(
            max_vx=float(self.declare_parameter("max_vx", 0.15).value),
            max_wz=float(self.declare_parameter("max_wz", 0.20).value),
            max_ax=float(self.declare_parameter("max_ax", 0.10).value),
            max_awz=float(self.declare_parameter("max_awz", 0.50).value),
            yaw_zero_epsilon=float(
                self.declare_parameter("yaw_zero_epsilon", 0.04).value),
            min_yaw_cmd=0.0,
        )
        self.mapping_limits = UdpMappingLimits(
            max_vx=self.command_limits.max_vx,
            max_wz=self.command_limits.max_wz,
            max_x=float(self.declare_parameter("udp_max_x", 0.50).value),
            max_yaw=float(self.declare_parameter("udp_max_yaw", 0.60).value),
            yaw_zero_epsilon=self.command_limits.yaw_zero_epsilon,
        )
        self.udp_yaw_slew_rate = max(
            0.0,
            float(self.declare_parameter(
                "udp_yaw_slew_rate", 2.0).value),
        )
        self.freshness_limits = FreshnessLimits(
            command=float(self.declare_parameter("command_timeout", 0.20).value),
            body_pose=float(self.declare_parameter("body_pose_timeout", 0.50).value),
            sensor_pose=float(
                self.declare_parameter("sensor_pose_timeout", 0.50).value),
            cloud=float(
                self.declare_parameter("front_cloud_timeout", 0.30).value),
            motion_info=0.0,
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
        self.preview_topic = self.declare_parameter(
            "preview_topic", "/scan/udp_axis_preview").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/scan/udp_safety_status").value
        self.navigation_enabled_topic = self.declare_parameter(
            "navigation_enabled_topic", "/scan/navigation_enabled").value
        self.arm_service_name = self.declare_parameter(
            "arm_service", "/scan/arm_udp_control").value

        self.inputs = InputState()
        self.sensor_cloud_pairs = RecentStampPairTracker(
            retention_sec=self.cloud_stamp_history_sec,
            tolerance_sec=self.freshness_limits.max_sensor_cloud_stamp_delta,
        )
        self.last_command = Command()
        self.preview_command = Command()
        self.preview_axis = UdpAxis()
        self.mapper = UdpCommandMapper(self.mapping_limits)
        self.state_machine = SafetyStateMachine()
        self.last_tick = time.monotonic()
        self.last_command_send = None
        self.last_preview_publish = None
        self.last_status_publish = None
        self.last_heartbeat_send = None
        self.last_heartbeat_ack = None
        self.last_heartbeat_error_code = None
        self.last_conflict_check = None
        self.conflicting_processes = []
        self.stop_reason = ""
        self.udp_error = ""
        self.udp_link = None
        self.ever_armed = False
        self.navigation_enabled = False
        self.manual_disarm_latched = False
        self.auto_arm_ready_since = None

        self.preview_pub = self.create_publisher(Twist, self.preview_topic, 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, self.status_topic, 10)
        self.navigation_enabled_pub = self.create_publisher(
            Bool, self.navigation_enabled_topic, NAVIGATION_STATE_QOS)
        self.command_sub = self.create_subscription(
            Twist, self.command_topic, self._command_callback,
            RELIABLE_SAFETY_QOS)
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

        if self.enable_udp_output:
            self._initialize_udp_link()

        self.timer = self.create_timer(1.0 / self.timer_rate, self._tick)
        mode = "REAL-CAPABLE DISARMED" if self.enable_udp_output else "DRY-RUN"
        self.get_logger().info(
            f"M20 UDP safety bridge started in {mode}; "
            f"target={self.udp_target_host}:{self.udp_target_port}; "
            f"motion_udp_enabled={self.enable_udp_output}; "
            f"auto_arm_enabled={self.auto_arm_enabled}")

    def _initialize_udp_link(self) -> bool:
        if self.udp_link is not None:
            return True
        try:
            self.udp_link = M20UdpLink(
                self.udp_target_host,
                self.udp_target_port,
                self.udp_bind_host,
                self.udp_bind_port,
            )
            self.udp_error = ""
            return True
        except OSError as exc:
            self.udp_error = str(exc)
            self.get_logger().error(f"Failed to initialize M20 UDP link: {exc}")
            return False

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
        self.sensor_cloud_pairs.add_pose(now, stamp_ns)
        self.inputs.sensor_cloud_pair_rx = (
            self.sensor_cloud_pairs.last_match_rx)

    def _front_cloud_callback(self, msg: Header) -> None:
        now = time.monotonic()
        stamp_ns = self._stamp_ns(msg.stamp)
        self.inputs.cloud_rx = now
        self.inputs.cloud_stamp_ns = stamp_ns
        self.sensor_cloud_pairs.add_cloud(now, stamp_ns)
        self.inputs.sensor_cloud_pair_rx = (
            self.sensor_cloud_pairs.last_match_rx)

    def _handle_arm(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self.manual_disarm_latched = True
            self.auto_arm_ready_since = None
            if self.state_machine.state is BridgeState.DISARMED:
                self._publish_navigation_enabled(False)
                response.success = True
                response.message = "already disarmed"
            else:
                self._begin_stop("manual_disarm")
                response.success = True
                response.message = "stopping with zero UDP sequence"
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

    def _arm_bridge(self, source: str) -> bool:
        if not self.state_machine.arm():
            return False
        self.ever_armed = True
        self.stop_reason = ""
        self.last_command_send = None
        self.auto_arm_ready_since = None
        self._publish_navigation_enabled(True)
        self.get_logger().warn(
            f"M20 UDP motion output ARMED ({source})")
        return True

    def _arm_blockers(self, now=None, force_conflict_check=True) -> list[str]:
        now = time.monotonic() if now is None else now
        blockers = []
        if not self.enable_udp_output:
            blockers.append("enable_udp_output_false")
        elif not self._initialize_udp_link():
            blockers.append("udp_link_unavailable")
        if self.state_machine.state is not BridgeState.DISARMED:
            blockers.append("bridge_not_disarmed")
        blockers.extend(self._health_reasons(now))
        if self.enable_udp_output:
            blockers.extend(self._udp_health_reasons(now))
            self._refresh_conflicts(now, force=force_conflict_check)
            if self.conflicting_processes:
                blockers.append(
                    "conflicting_processes:" + "|".join(
                        self.conflicting_processes))
        pending_command = limit_command(
            self.last_command.vx,
            self.last_command.wz,
            self.command_limits,
        )
        if any((
            abs(self.preview_command.vx) > 1e-6,
            abs(self.preview_command.wz)
            >= self.mapping_limits.yaw_zero_epsilon,
            abs(pending_command.vx) > 1e-6,
            abs(pending_command.wz)
            >= self.mapping_limits.yaw_zero_epsilon,
        )):
            blockers.append("command_not_zero")
        return blockers

    def _update_auto_arm(self, now: float, blockers: list[str]) -> bool:
        eligible = (
            self.auto_arm_enabled
            and self.enable_udp_output
            and not self.manual_disarm_latched
            and self.state_machine.state is BridgeState.DISARMED
        )
        if not eligible:
            self.auto_arm_ready_since = None
            return False

        if blockers:
            if self.auto_arm_ready_since is not None:
                self.get_logger().info(
                    "Automatic arm stability window cancelled: "
                    + ",".join(blockers))
            self.auto_arm_ready_since = None
            return False

        if self.auto_arm_ready_since is None:
            self.auto_arm_ready_since = now
            self.get_logger().info(
                "Automatic arm stability window started")

        if now - self.auto_arm_ready_since < self.auto_arm_stable_sec:
            return False

        return self._arm_bridge("automatic")

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.25)
        self.last_tick = now

        if self.enable_udp_output:
            self._poll_udp(now)
            self._send_heartbeat_if_due(now)
            self._refresh_conflicts(now)

        base_reasons = self._health_reasons(now)
        if base_reasons:
            self.preview_command = Command()
            self.mapper.reset()
            self.preview_axis = UdpAxis()
        else:
            target = limit_command(
                self.last_command.vx,
                self.last_command.wz,
                self.command_limits,
            )
            self.preview_command = slew_command(
                self.preview_command,
                target,
                dt,
                self.command_limits,
            )
            target_axis = self.mapper.map(
                self.preview_command.vx,
                self.preview_command.vy,
                self.preview_command.wz,
            )
            self.preview_axis = slew_udp_axis(
                self.preview_axis,
                target_axis,
                dt,
                self.udp_yaw_slew_rate,
            )
        if self._rate_due(now, self.last_preview_publish, self.preview_rate):
            self._publish_preview()
            self.last_preview_publish = now

        if self.state_machine.state is BridgeState.ARMED:
            armed_reasons = list(base_reasons)
            armed_reasons.extend(self._udp_health_reasons(now))
            if self.conflicting_processes:
                armed_reasons.append(
                    "conflicting_processes:" + "|".join(
                        self.conflicting_processes))
            if armed_reasons:
                self._begin_stop(",".join(armed_reasons))
            elif self._command_send_due(now):
                self._send_axis(self.preview_axis)
                self.last_command_send = now

        if self.state_machine.state is BridgeState.STOPPING:
            zero_sent = self._send_axis(UdpAxis())
            if zero_sent and self.state_machine.record_stop_publish():
                self.mapper.reset()
                self.preview_command = Command()
                self.preview_axis = UdpAxis()
                self.get_logger().warn(
                    f"M20 UDP output DISARMED: {self.stop_reason}")

        status_reasons = list(base_reasons)
        if self.enable_udp_output:
            status_reasons.extend(self._udp_health_reasons(now))
            if self.conflicting_processes:
                status_reasons.append(
                    "conflicting_processes:" + "|".join(
                        self.conflicting_processes))
        else:
            status_reasons.append("enable_udp_output_false")
        auto_arm_blockers = []
        if (
            self.auto_arm_enabled
            and self.enable_udp_output
            and not self.manual_disarm_latched
            and self.state_machine.state is BridgeState.DISARMED
        ):
            auto_arm_blockers = self._arm_blockers(
                now, force_conflict_check=False)
        self._update_auto_arm(
            now,
            auto_arm_blockers,
        )
        if self._rate_due(now, self.last_status_publish, self.status_rate):
            self._publish_status(status_reasons)
            self.last_status_publish = now

    def _health_reasons(self, now) -> list[str]:
        return health_reasons(
            now,
            self.inputs,
            self.freshness_limits,
            require_motion_info=False,
            use_completed_pair=True,
        )

    def _udp_health_reasons(self, now) -> list[str]:
        if self.udp_link is None:
            return ["udp_link_unavailable"]
        if self.last_heartbeat_ack is None:
            return ["heartbeat_missing"]
        if now - self.last_heartbeat_ack > self.heartbeat_timeout:
            return ["heartbeat_stale"]
        if self.last_heartbeat_error_code != 0:
            return [
                f"heartbeat_error:{self.last_heartbeat_error_code}"]
        return []

    def _poll_udp(self, now) -> None:
        if self.udp_link is None:
            return
        try:
            error_code = self.udp_link.poll_heartbeat_error()
        except OSError as exc:
            self.udp_error = str(exc)
            if self.state_machine.state is BridgeState.ARMED:
                self._begin_stop("udp_receive_error")
            return
        if error_code is not None:
            self.last_heartbeat_ack = now
            self.last_heartbeat_error_code = error_code
            self.udp_error = ""

    def _send_heartbeat_if_due(self, now) -> None:
        if self.udp_link is None:
            return
        period = 1.0 / max(self.heartbeat_rate, 0.1)
        if (
            self.last_heartbeat_send is not None
            and now - self.last_heartbeat_send < period
        ):
            return
        try:
            self.udp_link.send_heartbeat()
            self.last_heartbeat_send = now
            self.udp_error = ""
        except OSError as exc:
            self.udp_error = str(exc)
            if self.state_machine.state is BridgeState.ARMED:
                self._begin_stop("udp_heartbeat_send_error")

    def _send_axis(self, axis: UdpAxis) -> bool:
        if self.udp_link is None:
            return False
        try:
            self.udp_link.send_axis(axis)
            self.udp_error = ""
            return True
        except OSError as exc:
            self.udp_error = str(exc)
            if self.state_machine.state is BridgeState.ARMED:
                self._begin_stop("udp_axis_send_error")
            return False

    def _command_send_due(self, now) -> bool:
        return self._rate_due(
            now, self.last_command_send, self.command_rate)

    @staticmethod
    def _rate_due(now, last_publish, rate) -> bool:
        if last_publish is None:
            return True
        return now - last_publish >= (1.0 / max(rate, 0.1))

    def _refresh_conflicts(self, now, force=False) -> None:
        if (
            not force
            and self.last_conflict_check is not None
            and now - self.last_conflict_check < self.conflict_check_period
        ):
            return
        self.conflicting_processes = find_conflicting_processes(
            self.conflicting_process_names)
        self.last_conflict_check = now

    def _begin_stop(self, reason: str) -> None:
        self.auto_arm_ready_since = None
        if self.state_machine.state is BridgeState.DISARMED:
            self._publish_navigation_enabled(False)
            return
        self._publish_navigation_enabled(False)
        if self.state_machine.state is not BridgeState.STOPPING:
            self.stop_reason = reason
            self.get_logger().error(f"Disarming M20 UDP output: {reason}")
            self.mapper.reset()
            self.preview_command = Command()
            self.preview_axis = UdpAxis()
        self.state_machine.begin_stop(self.stop_cycles)

    def _publish_navigation_enabled(self, enabled: bool) -> None:
        self.navigation_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.navigation_enabled_pub.publish(msg)

    def _publish_preview(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.preview_axis.x)
        msg.linear.y = 0.0
        msg.angular.z = float(self.preview_axis.yaw)
        self.preview_pub.publish(msg)

    def _publish_status(self, reasons: list[str]) -> None:
        status = DiagnosticStatus()
        status.name = "scan_m20_udp_safety_bridge"
        status.hardware_id = "m20"
        status.level = DiagnosticStatus.OK if not reasons else DiagnosticStatus.WARN
        status.message = "ready" if not reasons else ",".join(reasons)
        heartbeat_age = ""
        if self.last_heartbeat_ack is not None:
            heartbeat_age = f"{time.monotonic() - self.last_heartbeat_ack:.3f}"
        auto_arm_ready_age = ""
        if self.auto_arm_ready_since is not None:
            auto_arm_ready_age = (
                f"{time.monotonic() - self.auto_arm_ready_since:.3f}")
        status.values = [
            KeyValue(key="state", value=self.state_machine.state.value),
            KeyValue(
                key="enable_udp_output",
                value=str(self.enable_udp_output).lower()),
            KeyValue(
                key="navigation_enabled",
                value=str(self.navigation_enabled).lower()),
            KeyValue(
                key="auto_arm_enabled",
                value=str(self.auto_arm_enabled).lower()),
            KeyValue(
                key="manual_disarm_latched",
                value=str(self.manual_disarm_latched).lower()),
            KeyValue(
                key="auto_arm_ready_age_sec",
                value=auto_arm_ready_age),
            KeyValue(
                key="udp_target",
                value=f"{self.udp_target_host}:{self.udp_target_port}"),
            KeyValue(key="heartbeat_age_sec", value=heartbeat_age),
            KeyValue(
                key="heartbeat_error_code",
                value="" if self.last_heartbeat_error_code is None else str(
                    self.last_heartbeat_error_code)),
            KeyValue(
                key="conflicting_processes",
                value="|".join(self.conflicting_processes)),
            KeyValue(key="udp_x", value=f"{self.preview_axis.x:.3f}"),
            KeyValue(key="udp_yaw", value=f"{self.preview_axis.yaw:.3f}"),
            KeyValue(
                key="stop_cycles_remaining",
                value=str(self.state_machine.stop_cycles_remaining)),
            KeyValue(key="stop_reason", value=self.stop_reason),
            KeyValue(key="udp_error", value=self.udp_error),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def destroy_node(self):
        self._publish_navigation_enabled(False)
        if self.udp_link is not None:
            if self.ever_armed:
                period = 1.0 / max(self.timer_rate, 1.0)
                for _ in range(max(1, self.stop_cycles)):
                    try:
                        self.udp_link.send_axis(UdpAxis())
                    except OSError:
                        break
                    time.sleep(period)
            self.udp_link.close()
            self.udp_link = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanM20UdpSafetyBridge()
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
