# M20 Safety Freshness Timeout Adjustment Design

## Goal

Allow the UDP safety bridge to tolerate bounded front-cloud and sensor-pose
delivery jitter without disabling stale-input protection.

## Decision

Change only these parameters in `m20_scan_udp_bridge.yaml`:

- `front_cloud_timeout`: `0.30` seconds to `0.80` seconds.
- `sensor_pose_timeout`: `0.50` seconds to `0.80` seconds.

Both values must change because completed-pair freshness uses the stricter of
the two limits.

The selected 0.80-second limit covers the previously observed 0.703-second
delivery gap with a small margin. It also remains bounded: a continuing outage
still rejects arming or triggers automatic stopping after 0.80 seconds.

## Unchanged Safety Checks

- Keep `max_sensor_cloud_stamp_delta` at 0.02 seconds.
- Keep command, body-pose, heartbeat, conflict, zero-command, and UDP checks
  unchanged.
- Keep completed timestamp-pair matching enabled.
- Do not bypass the safety bridge.

## Impact

The bridge may continue accepting the last completed sensor/cloud pair for up
to 0.80 seconds. This is a longer blind interval than the current 0.30-second
cloud limit, so the adjustment is intentionally limited to the two observed
sensor-path timeouts and does not relax command or heartbeat protection.

## Testing and Runtime Validation

- Add a configuration test asserting both values are 0.80 seconds.
- Run the full `m20_scan_bringup` Pytest and selected-package Colcon tests.
- Rebuild `m20_scan_bringup`.
- Explicitly disarm before restarting the UDP safety bridge.
- Confirm D435i and both Foxglove bridges remain stopped.
- Confirm zero command and `ready` status.
- Perform a 12-second zero-speed arm test while monitoring stale, mismatch, and
  nonzero-command events.
- Explicitly disarm and require final state `DISARMED`.

Rollback restores `front_cloud_timeout: 0.30` and
`sensor_pose_timeout: 0.50`.
