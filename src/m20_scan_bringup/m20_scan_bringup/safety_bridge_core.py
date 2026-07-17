from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Optional


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


class RecentStampHistory:
    def __init__(self, retention_sec: float, max_entries: int = 64) -> None:
        if retention_sec <= 0.0:
            raise ValueError("retention_sec must be positive")
        if max_entries <= 0:
            raise ValueError("max_entries must be positive")
        self.retention_sec = float(retention_sec)
        self._entries = deque(maxlen=int(max_entries))

    def add(self, received_at: float, stamp_ns: int) -> None:
        self._prune(received_at)
        self._entries.append((float(received_at), int(stamp_ns)))

    def stamps(self, now: float) -> tuple[int, ...]:
        self._prune(now)
        return tuple(stamp_ns for _, stamp_ns in self._entries)

    def _prune(self, now: float) -> None:
        cutoff = float(now) - self.retention_sec
        while self._entries and self._entries[0][0] < cutoff:
            self._entries.popleft()


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
    cloud_stamp_history_ns: Optional[Iterable[int]] = None,
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

    if cloud_stamp_history_ns is None:
        cloud_stamps = (
            () if inputs.cloud_stamp_ns is None
            else (inputs.cloud_stamp_ns,)
        )
    else:
        cloud_stamps = tuple(cloud_stamp_history_ns)

    if inputs.sensor_pose_stamp_ns is None or not cloud_stamps:
        reasons.append("sensor_cloud_stamp_missing")
    else:
        tolerance_ns = limits.max_sensor_cloud_stamp_delta * 1e9
        matches_recent_cloud = any(
            abs(inputs.sensor_pose_stamp_ns - cloud_stamp_ns) <= tolerance_ns
            for cloud_stamp_ns in cloud_stamps
        )
        if not matches_recent_cloud:
            reasons.append("sensor_cloud_stamp_mismatch")
    return reasons


def _approach(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)
