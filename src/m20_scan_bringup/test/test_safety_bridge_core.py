import math

from m20_scan_bringup.safety_bridge_core import (
    BridgeState,
    Command,
    CommandLimits,
    FreshnessLimits,
    InputState,
    RecentStampHistory,
    RecentStampPairTracker,
    SafetyStateMachine,
    health_reasons,
    limit_command,
    slew_command,
)


LIMITS = CommandLimits(
    max_vx=0.05,
    max_wz=0.30,
    max_ax=0.10,
    max_awz=0.50,
    yaw_zero_epsilon=0.01,
    min_yaw_cmd=0.15,
)

FRESHNESS = FreshnessLimits(
    command=0.20,
    body_pose=0.50,
    sensor_pose=0.50,
    cloud=0.30,
    motion_info=0.50,
    max_sensor_cloud_stamp_delta=0.02,
)


def test_command_mapping_rejects_reverse_lateral_and_limits_yaw():
    command = limit_command(-0.2, 0.8, LIMITS)
    assert command == Command(vx=0.0, vy=0.0, wz=0.30)


def test_command_mapping_applies_yaw_deadzone_and_minimum():
    assert limit_command(0.02, 0.005, LIMITS).wz == 0.0
    assert limit_command(0.02, -0.02, LIMITS).wz == -0.15


def test_nonfinite_commands_become_zero():
    assert limit_command(math.nan, math.inf, LIMITS) == Command()


def test_acceleration_limits_are_applied():
    command = slew_command(Command(), Command(vx=0.05, wz=0.30), 0.1, LIMITS)
    assert math.isclose(command.vx, 0.01)
    assert math.isclose(command.wz, 0.05)


def test_health_accepts_fresh_same_stamp_inputs():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        motion_info_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
        cloud_stamp_ns=1_010_000_000,
    )
    assert health_reasons(10.1, inputs, FRESHNESS, require_motion_info=True) == []


def test_health_reports_stale_and_stamp_mismatch():
    inputs = InputState(
        command_rx=9.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
        cloud_stamp_ns=1_100_000_000,
    )
    reasons = health_reasons(10.1, inputs, FRESHNESS, require_motion_info=False)
    assert "command_stale" in reasons
    assert "sensor_cloud_stamp_mismatch" in reasons


def test_health_accepts_sensor_stamp_matching_recent_cloud_history():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
        cloud_stamp_ns=1_200_000_000,
    )
    reasons = health_reasons(
        10.1,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        cloud_stamp_history_ns=(1_000_000_000, 1_200_000_000),
    )
    assert "sensor_cloud_stamp_mismatch" not in reasons
    assert "sensor_cloud_stamp_missing" not in reasons


def test_health_reports_when_sensor_stamp_matches_no_recent_cloud():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
        cloud_stamp_ns=1_200_000_000,
    )
    reasons = health_reasons(
        10.1,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        cloud_stamp_history_ns=(1_100_000_000, 1_200_000_000),
    )
    assert "sensor_cloud_stamp_mismatch" in reasons


def test_health_reports_missing_for_empty_cloud_history():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
    )
    reasons = health_reasons(
        10.1,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        cloud_stamp_history_ns=(),
    )
    assert "sensor_cloud_stamp_missing" in reasons


def test_recent_stamp_history_prunes_expired_entries():
    history = RecentStampHistory(retention_sec=1.0, max_entries=3)
    history.add(received_at=8.0, stamp_ns=800)
    history.add(received_at=9.5, stamp_ns=950)
    history.add(received_at=10.0, stamp_ns=1000)
    assert history.stamps(now=10.1) == (950, 1000)


def test_stamp_pair_tracker_matches_both_arrival_orders():
    stamp_first = RecentStampPairTracker(
        retention_sec=1.0, tolerance_sec=0.02)
    assert not stamp_first.add_cloud(received_at=10.0, stamp_ns=1_000_000_000)
    assert stamp_first.add_pose(received_at=10.1, stamp_ns=1_010_000_000)
    assert stamp_first.last_match_rx == 10.1

    pose_first = RecentStampPairTracker(
        retention_sec=1.0, tolerance_sec=0.02)
    assert not pose_first.add_pose(received_at=20.0, stamp_ns=2_000_000_000)
    assert pose_first.add_cloud(received_at=20.1, stamp_ns=2_010_000_000)
    assert pose_first.last_match_rx == 20.1


def test_stamp_pair_tracker_does_not_reuse_or_refresh_old_match():
    tracker = RecentStampPairTracker(
        retention_sec=1.0, tolerance_sec=0.02)
    tracker.add_pose(received_at=10.0, stamp_ns=1_000_000_000)
    assert tracker.add_cloud(received_at=10.1, stamp_ns=1_000_000_000)

    assert not tracker.add_cloud(received_at=10.2, stamp_ns=1_000_000_000)
    assert not tracker.add_pose(received_at=10.3, stamp_ns=2_000_000_000)
    assert not tracker.add_cloud(received_at=10.4, stamp_ns=2_100_000_000)
    assert tracker.last_match_rx == 10.1


def test_stamp_pair_tracker_rejects_outside_tolerance_and_expired_entries():
    tracker = RecentStampPairTracker(
        retention_sec=1.0, tolerance_sec=0.02)
    tracker.add_pose(received_at=8.0, stamp_ns=1_000_000_000)
    assert not tracker.add_cloud(
        received_at=9.1, stamp_ns=1_000_000_000)

    tracker.add_pose(received_at=9.2, stamp_ns=2_000_000_000)
    assert not tracker.add_cloud(
        received_at=9.3, stamp_ns=2_021_000_000)
    assert tracker.last_match_rx is None


def test_completed_pair_health_is_fresh_then_fails_closed_when_stale():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=2_000_000_000,
        cloud_stamp_ns=2_100_000_000,
        sensor_cloud_pair_rx=10.0,
    )
    reasons = health_reasons(
        10.1,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        use_completed_pair=True,
    )
    assert "sensor_cloud_stamp_mismatch" not in reasons

    reasons = health_reasons(
        10.31,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        use_completed_pair=True,
    )
    assert "sensor_cloud_stamp_mismatch" in reasons


def test_completed_pair_health_rejects_when_no_pair_has_completed():
    inputs = InputState(
        command_rx=10.0,
        body_pose_rx=10.0,
        sensor_pose_rx=10.0,
        cloud_rx=10.0,
        sensor_pose_stamp_ns=1_000_000_000,
        cloud_stamp_ns=1_100_000_000,
    )
    reasons = health_reasons(
        10.1,
        inputs,
        FRESHNESS,
        require_motion_info=False,
        use_completed_pair=True,
    )
    assert "sensor_cloud_stamp_mismatch" in reasons


def test_stop_sequence_requires_exact_zero_publish_count():
    machine = SafetyStateMachine()
    assert machine.arm()
    machine.begin_stop(5)
    assert machine.state is BridgeState.STOPPING
    for _ in range(4):
        assert not machine.record_stop_publish()
    assert machine.record_stop_publish()
    assert machine.state is BridgeState.DISARMED
