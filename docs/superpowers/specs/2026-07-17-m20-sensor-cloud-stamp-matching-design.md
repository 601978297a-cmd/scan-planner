# M20 Sensor/Cloud Timestamp Matching Design

Date: 2026-07-17

## Problem

The UDP safety bridge currently compares only the latest
`/scan/sensor_pose` timestamp with the latest
`/scan/front_cloud_stamp` timestamp.

The sensor-pose adapter publishes the raw cloud header before it completes
body-pose matching and publishes the corresponding sensor pose. During that
short interval, the latest cloud belongs to sample `t2` while the latest sensor
pose still belongs to sample `t1`. At normal lidar rates, `t2 - t1` is much
larger than the 20 ms synchronization tolerance. A 20 Hz safety tick can
therefore report `sensor_cloud_stamp_mismatch` and disarm even though the `t2`
sensor pose arrives shortly afterward.

## Scope

This change only corrects the false latest-to-latest mismatch. It does not
relax the existing command, body-pose, sensor-pose, cloud, heartbeat, process
conflict, or zero-stop safety checks.

## Design

The UDP bridge maintains a bounded history of received cloud timestamps:

- Each entry contains the monotonic receive time and ROS cloud timestamp.
- Entries are retained for 1.0 second.
- The collection is also bounded by a fixed maximum length.
- Pruning uses monotonic receive time so ROS clock changes cannot retain stale
  entries indefinitely.

The latest sensor-pose timestamp is compared with every retained cloud
timestamp:

- If any absolute difference is at most 20 ms, synchronization is healthy.
- If both inputs exist but no retained cloud timestamp matches, report
  `sensor_cloud_stamp_mismatch`.
- If either input has not yet been received, report
  `sensor_cloud_stamp_missing`.

The existing wall-time freshness checks remain authoritative:

- A stale body pose still reports `body_pose_stale`.
- A stale sensor pose still reports `sensor_pose_stale`.
- A stale front cloud still reports `front_cloud_stale`.

Matching an older entry in the one-second history cannot conceal a dead input,
because its independent freshness check still fails.

## Implementation Boundaries

- Keep `/scan/front_cloud_stamp` and its lightweight `std_msgs/msg/Header`
  interface unchanged.
- Add a pure timestamp-history matching helper in `safety_bridge_core.py`.
- Use the helper only from `scan_m20_udp_safety_bridge.py`.
- Preserve the current latest-to-latest behavior for the existing `/NAV_CMD`
  bridge unless it explicitly supplies timestamp history.
- Add `cloud_stamp_history_sec: 1.0` to the UDP bridge configuration.

## Tests

Unit tests must cover:

1. Latest cloud `t2`, sensor pose `t1`, and history containing both `t1` and
   `t2`: healthy.
2. Sensor pose does not match any recent cloud: mismatch.
3. Missing sensor pose or empty cloud history: missing.
4. Expired cloud history entries are pruned.
5. Real sensor-pose and cloud wall-time staleness still reports stale.
6. Existing safety bridge and UDP protocol tests continue to pass.

## Runtime Acceptance

With Lightning, the front lidar, SCAN dry-run, and the UDP backend running:

- Normal message arrival order does not produce transient
  `sensor_cloud_stamp_mismatch`.
- Stopping either sensor-pose or cloud updates still produces the corresponding
  stale diagnostic.
- `enable_udp_output` remains `false`.
- The bridge remains `DISARMED`.
- `/NAV_CMD` still has zero publishers.
