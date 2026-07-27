import math
import os
import socket
import time

from m20_scan_bringup.m20_udp_protocol import (
    M20UdpLink,
    PACKET_HEADER_SIZE,
    PACKET_MAGIC,
    UdpAxis,
    UdpCommandMapper,
    UdpMappingLimits,
    build_axis_packet,
    build_heartbeat_packet,
    build_motion_state_packet,
    build_packet,
    find_conflicting_processes,
    heartbeat_error_code,
    parse_packet,
    scale_direct_udp_axis,
    slew_udp_axis,
)


LIMITS = UdpMappingLimits(
    max_vx=0.15,
    max_wz=0.20,
    max_x=0.50,
    max_yaw=0.60,
    yaw_zero_epsilon=0.04,
)


def test_packet_matches_hqs_header_and_json_layout():
    packet = build_axis_packet(
        UdpAxis(x=0.25, y=-0.20, yaw=-0.75),
        timestamp="2026-07-17 12:00:00",
    )
    assert packet[:4] == PACKET_MAGIC
    payload_size = int.from_bytes(packet[4:6], "little")
    assert payload_size == len(packet) - PACKET_HEADER_SIZE

    patrol = parse_packet(packet)["PatrolDevice"]
    assert patrol["Type"] == 2
    assert patrol["Command"] == 21
    assert patrol["Items"] == {
        "X": 0.25,
        "Y": -0.20,
        "Z": 0,
        "Roll": 0,
        "Pitch": 0,
        "Yaw": -0.75,
    }


def test_direct_udp_scaling_matches_super_lio_and_clamps():
    axis = scale_direct_udp_axis(
        0.15, -0.20, 0.20, 3.0, 3.0, 2.0)
    assert math.isclose(axis.x, 0.45)
    assert math.isclose(axis.y, -0.60)
    assert math.isclose(axis.yaw, 0.40)

    assert scale_direct_udp_axis(
        2.0, -2.0, 2.0, 3.0, 3.0, 2.0
    ) == UdpAxis(x=1.0, y=-1.0, yaw=3.0)
    assert scale_direct_udp_axis(
        math.nan, 0.0, 0.0, 3.0, 3.0, 2.0
    ) == UdpAxis()


def test_direct_udp_scaling_maps_half_meter_per_second_to_full_axis():
    assert scale_direct_udp_axis(
        0.5, -0.5, 0.2, 2.0, 2.0, 5.0
    ) == UdpAxis(x=1.0, y=-1.0, yaw=1.0)


def test_motion_state_packet_matches_super_lio_stand_command():
    packet = build_motion_state_packet(
        1, timestamp="2026-07-24 12:00:00")
    patrol = parse_packet(packet)["PatrolDevice"]

    assert patrol["Type"] == 2
    assert patrol["Command"] == 22
    assert patrol["Items"] == {"MotionParam": 1}


def test_heartbeat_ack_parser_accepts_only_matching_response():
    ack = build_packet(
        100,
        100,
        {"ErrorCode": 0},
        timestamp="2026-07-17 12:00:00",
    )
    assert heartbeat_error_code(ack) == 0
    assert heartbeat_error_code(build_heartbeat_packet()) is None
    assert heartbeat_error_code(build_packet(2002, 1, {"ErrorCode": 0})) is None
    assert heartbeat_error_code(b"invalid") is None


def test_mapper_rejects_reverse_lateral_and_nonfinite_values():
    mapper = UdpCommandMapper(LIMITS)
    assert mapper.map(-0.5, 0.8, 0.0) == UdpAxis()
    assert mapper.map(math.nan, 0.0, 0.2) == UdpAxis()
    assert mapper.map(0.05, 0.0, math.inf) == UdpAxis()


def test_mapper_scales_forward_and_saturates():
    mapper = UdpCommandMapper(LIMITS)
    assert mapper.map(0.075, 0.0, 0.0).x == 0.25
    assert mapper.map(0.15, 0.0, 0.0).x == 0.50
    assert mapper.map(1.0, 0.0, 0.0).x == 0.50


def test_mapper_scales_yaw_linearly_after_zero_epsilon():
    mapper = UdpCommandMapper(LIMITS)
    assert mapper.map(0.0, 0.0, 0.03).yaw == 0.0
    assert math.isclose(mapper.map(0.0, 0.0, 0.04).yaw, 0.12)
    assert math.isclose(mapper.map(0.0, 0.0, 0.10).yaw, 0.30)
    assert math.isclose(mapper.map(0.0, 0.0, 0.20).yaw, 0.60)
    assert math.isclose(mapper.map(0.0, 0.0, -0.10).yaw, -0.30)
    assert math.isclose(mapper.map(0.0, 0.0, -1.00).yaw, -0.60)


def test_udp_yaw_slew_ramps_without_overshoot():
    first = slew_udp_axis(
        UdpAxis(), UdpAxis(x=0.25, yaw=0.60), 0.05, 2.0)
    assert first == UdpAxis(x=0.25, yaw=0.10)

    target = UdpAxis(x=0.25, yaw=0.60)
    final = slew_udp_axis(UdpAxis(yaw=0.55), target, 0.05, 2.0)
    assert final == target


def test_udp_yaw_reversal_outputs_zero_before_opposite_sign():
    target = UdpAxis(yaw=-0.60)
    current = UdpAxis(yaw=0.60)
    outputs = []

    for _ in range(8):
        current = slew_udp_axis(current, target, 0.05, 2.0)
        outputs.append(current.yaw)

    first_negative = next(
        index for index, yaw in enumerate(outputs) if yaw < 0.0)
    assert 0.0 in outputs[:first_negative]
    assert outputs[first_negative] == -0.10


def test_udp_yaw_slew_reset_starts_again_from_zero():
    target = UdpAxis(yaw=0.75)
    moving = slew_udp_axis(UdpAxis(), target, 0.05, 2.0)
    assert moving.yaw == 0.10

    reset = UdpAxis()
    restarted = slew_udp_axis(reset, target, 0.05, 2.0)
    assert restarted.yaw == 0.10


def test_udp_link_round_trip_with_local_heartbeat_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(1.0)
    link = M20UdpLink("127.0.0.1", server.getsockname()[1])
    try:
        link.send_heartbeat()
        request, client_address = server.recvfrom(8192)
        patrol = parse_packet(request)["PatrolDevice"]
        assert (patrol["Type"], patrol["Command"]) == (100, 100)

        server.sendto(build_packet(100, 100, {"ErrorCode": 0}), client_address)
        deadline = time.monotonic() + 1.0
        result = None
        while result is None and time.monotonic() < deadline:
            result = link.poll_heartbeat_error()
            time.sleep(0.01)
        assert result == 0
    finally:
        link.close()
        server.close()


def test_conflict_detection_reads_process_cmdlines(tmp_path):
    proc_root = tmp_path / "proc"
    proc_root.mkdir()
    for pid, cmdline in (
        ("101", b"./key_test\0"),
        ("102", b"python3\0/home/nvidia/m20_udp_drive_test.py\0"),
        ("103", b"python3\0unrelated.py\0"),
    ):
        process_dir = proc_root / pid
        process_dir.mkdir()
        (process_dir / "cmdline").write_bytes(cmdline)

    assert find_conflicting_processes(
        ("key_test", "m20_udp_drive_test.py"),
        proc_root=str(proc_root),
        own_pid=os.getpid(),
    ) == ["key_test", "m20_udp_drive_test.py"]
