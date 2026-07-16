from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class CommandLimits:
    max_vx: float
    max_wz: float
    max_ax: float
    max_awz: float
    yaw_zero_epsilon: float
    min_yaw_cmd: float


@dataclass
class InputState:
    command_rx: Optional[float] = None
    body_pose_rx: Optional[float] = None
    sensor_pose_rx: Optional[float] = None
    cloud_rx: Optional[float] = None
    motion_info_rx: Optional[float] = None
    sensor_pose_stamp_ns: Optional[int] = None
    cloud_stamp_ns: Optional[int] = None


@dataclass(frozen=True)
class FreshnessLimits:
    command: float
    body_pose: float
    sensor_pose: float
    cloud: float
    motion_info: float
    max_sensor_cloud_stamp_delta: float


class BridgeState(Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    STOPPING = "STOPPING"


class SafetyStateMachine:
    def __init__(self) -> None:
        self.state = BridgeState.DISARMED
        self.stop_cycles_remaining = 0

    def arm(self) -> bool:
        if self.state is not BridgeState.DISARMED:
            return False
        self.state = BridgeState.ARMED
        return True

    def begin_stop(self, cycles: int) -> None:
        if self.state is BridgeState.DISARMED:
            return
        self.state = BridgeState.STOPPING
        self.stop_cycles_remaining = max(1, int(cycles))

    def record_stop_publish(self) -> bool:
        if self.state is not BridgeState.STOPPING:
            return False
        self.stop_cycles_remaining -= 1
        if self.stop_cycles_remaining <= 0:
            self.state = BridgeState.DISARMED
            return True
        return False


def limit_command(vx: float, wz: float, limits: CommandLimits) -> Command:
    vx = vx if math.isfinite(vx) else 0.0
    wz = wz if math.isfinite(wz) else 0.0

    safe_vx = min(max(vx, 0.0), limits.max_vx)
    if abs(wz) <= limits.yaw_zero_epsilon:
        safe_wz = 0.0
    else:
        magnitude = max(abs(wz), limits.min_yaw_cmd)
        safe_wz = math.copysign(min(magnitude, limits.max_wz), wz)
    return Command(vx=safe_vx, vy=0.0, wz=safe_wz)


def slew_command(current: Command, target: Command, dt: float, limits: CommandLimits) -> Command:
    dt = max(0.0, dt)
    return Command(
        vx=_approach(current.vx, target.vx, limits.max_ax * dt),
        vy=0.0,
        wz=_approach(current.wz, target.wz, limits.max_awz * dt),
    )


def health_reasons(
    now: float,
    inputs: InputState,
    limits: FreshnessLimits,
    require_motion_info: bool,
) -> list[str]:
    checks = [
        ("command", inputs.command_rx, limits.command),
        ("body_pose", inputs.body_pose_rx, limits.body_pose),
        ("sensor_pose", inputs.sensor_pose_rx, limits.sensor_pose),
        ("front_cloud", inputs.cloud_rx, limits.cloud),
    ]
    if require_motion_info:
        checks.append(("motion_info", inputs.motion_info_rx, limits.motion_info))

    reasons = []
    for name, received_at, timeout in checks:
        if received_at is None:
            reasons.append(f"{name}_missing")
        elif now - received_at > timeout:
            reasons.append(f"{name}_stale")

    if inputs.sensor_pose_stamp_ns is None or inputs.cloud_stamp_ns is None:
        reasons.append("sensor_cloud_stamp_missing")
    else:
        delta = abs(inputs.sensor_pose_stamp_ns - inputs.cloud_stamp_ns) / 1e9
        if delta > limits.max_sensor_cloud_stamp_delta:
            reasons.append("sensor_cloud_stamp_mismatch")
    return reasons


def _approach(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)
