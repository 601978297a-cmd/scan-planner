from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
import socket
from typing import Iterable, Optional


PACKET_MAGIC = bytes((0xEB, 0x91, 0xEB, 0x90))
PACKET_TAIL = bytes((0x01, 0x00, 0x01)) + bytes(7)
PACKET_HEADER_SIZE = 16


@dataclass(frozen=True)
class UdpAxis:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class UdpMappingLimits:
    max_vx: float
    max_wz: float
    max_x: float
    max_yaw: float
    yaw_zero_epsilon: float


class UdpCommandMapper:
    def __init__(self, limits: UdpMappingLimits):
        self.limits = limits

    def reset(self) -> None:
        pass

    def map(self, vx: float, _vy: float, wz: float) -> UdpAxis:
        if not all(math.isfinite(value) for value in (vx, _vy, wz)):
            self.reset()
            return UdpAxis()

        x = 0.0
        if vx > 0.0 and self.limits.max_vx > 0.0:
            x_ratio = min(vx / self.limits.max_vx, 1.0)
            x = self.limits.max_x * x_ratio

        yaw = self._map_yaw(wz)
        return UdpAxis(x=x, y=0.0, yaw=yaw)

    def _map_yaw(self, wz: float) -> float:
        magnitude = abs(wz)
        if magnitude < self.limits.yaw_zero_epsilon:
            return 0.0
        if self.limits.max_wz <= 0.0 or self.limits.max_yaw <= 0.0:
            return 0.0

        ratio = min(magnitude / self.limits.max_wz, 1.0)
        return math.copysign(self.limits.max_yaw * ratio, wz)


def slew_udp_axis(
    current: UdpAxis,
    target: UdpAxis,
    dt: float,
    max_yaw_rate: float,
) -> UdpAxis:
    values = (
        current.x,
        current.y,
        current.yaw,
        target.x,
        target.y,
        target.yaw,
        dt,
        max_yaw_rate,
    )
    if not all(math.isfinite(value) for value in values):
        return UdpAxis()

    max_delta = max(0.0, dt) * max(0.0, max_yaw_rate)
    desired_yaw = target.yaw
    if current.yaw * target.yaw < 0.0:
        desired_yaw = 0.0

    delta = desired_yaw - current.yaw
    if abs(delta) <= max_delta:
        yaw = desired_yaw
    elif max_delta > 0.0:
        yaw = current.yaw + math.copysign(max_delta, delta)
    else:
        yaw = current.yaw
    return UdpAxis(x=target.x, y=target.y, yaw=yaw)


def build_packet(
    message_type: int,
    command: int,
    items: Optional[dict] = None,
    timestamp: Optional[str] = None,
) -> bytes:
    body = json.dumps(
        {
            "PatrolDevice": {
                "Type": int(message_type),
                "Command": int(command),
                "Time": timestamp or datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Items": items or {},
            }
        },
        separators=(",", ":"),
    ).encode("utf-8")
    if len(body) > 0xFFFF:
        raise ValueError("UDP JSON payload exceeds protocol length field")
    return (
        PACKET_MAGIC
        + len(body).to_bytes(2, "little")
        + PACKET_TAIL
        + body
    )


def build_heartbeat_packet(timestamp: Optional[str] = None) -> bytes:
    return build_packet(100, 100, timestamp=timestamp)


def build_axis_packet(axis: UdpAxis, timestamp: Optional[str] = None) -> bytes:
    values = (axis.x, axis.y, axis.yaw)
    if not all(math.isfinite(value) for value in values):
        axis = UdpAxis()
    return build_packet(
        2,
        21,
        {
            "X": round(float(axis.x), 3),
            "Y": 0.0,
            "Z": 0,
            "Roll": 0,
            "Pitch": 0,
            "Yaw": round(float(axis.yaw), 3),
        },
        timestamp=timestamp,
    )


def parse_packet(packet: bytes) -> dict:
    if len(packet) < PACKET_HEADER_SIZE:
        raise ValueError("UDP packet is shorter than the protocol header")
    if packet[:4] != PACKET_MAGIC:
        raise ValueError("UDP packet has an invalid magic value")
    payload_size = int.from_bytes(packet[4:6], "little")
    if len(packet) < PACKET_HEADER_SIZE + payload_size:
        raise ValueError("UDP packet payload is truncated")
    payload = packet[
        PACKET_HEADER_SIZE:PACKET_HEADER_SIZE + payload_size]
    decoded = json.loads(payload.decode("utf-8"))
    if not isinstance(decoded, dict) or "PatrolDevice" not in decoded:
        raise ValueError("UDP packet does not contain PatrolDevice")
    return decoded


def heartbeat_error_code(packet: bytes) -> Optional[int]:
    try:
        patrol = parse_packet(packet)["PatrolDevice"]
        if int(patrol.get("Type", -1)) != 100:
            return None
        if int(patrol.get("Command", -1)) != 100:
            return None
        return int(patrol.get("Items", {}).get("ErrorCode"))
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


class M20UdpLink:
    def __init__(
        self,
        target_host: str,
        target_port: int,
        bind_host: str = "",
        bind_port: int = 0,
    ):
        self.target = (target_host, int(target_port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind_host, int(bind_port)))
        self.socket.setblocking(False)

    @property
    def local_address(self):
        return self.socket.getsockname()

    def send_heartbeat(self) -> None:
        self.socket.sendto(build_heartbeat_packet(), self.target)

    def send_axis(self, axis: UdpAxis) -> None:
        self.socket.sendto(build_axis_packet(axis), self.target)

    def poll_heartbeat_error(self) -> Optional[int]:
        latest = None
        while True:
            try:
                packet, address = self.socket.recvfrom(8192)
            except BlockingIOError:
                return latest
            if address != self.target:
                continue
            error_code = heartbeat_error_code(packet)
            if error_code is not None:
                latest = error_code

    def close(self) -> None:
        self.socket.close()


def find_conflicting_processes(
    names: Iterable[str],
    proc_root: str = "/proc",
    own_pid: Optional[int] = None,
) -> list[str]:
    wanted = set(names)
    found = set()
    own_pid = os.getpid() if own_pid is None else own_pid

    try:
        entries = os.listdir(proc_root)
    except OSError:
        return []

    for entry in entries:
        if not entry.isdigit() or int(entry) == own_pid:
            continue
        try:
            with open(
                os.path.join(proc_root, entry, "cmdline"), "rb"
            ) as handle:
                tokens = [
                    token.decode("utf-8", errors="ignore")
                    for token in handle.read().split(b"\0")
                    if token
                ]
        except OSError:
            continue

        basenames = {os.path.basename(token) for token in tokens}
        found.update(wanted.intersection(basenames))

    return sorted(found)
