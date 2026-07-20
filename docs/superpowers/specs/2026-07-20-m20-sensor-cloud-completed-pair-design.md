# M20 Sensor/Cloud Completed-Pair Safety Design

## Goal

Make UDP arming independent of the DDS arrival order of
`/scan/sensor_pose` and `/scan/front_cloud_stamp`, while preserving the
existing 20 ms timestamp tolerance and fail-closed behavior.

## Observed Failure

The adapter now calls the stamp publisher before the pose publisher, but the
UDP safety bridge can still receive the pose callback first because the two
messages use separate DDS writers. The bridge currently compares the newest
pose stamp with its cloud-stamp history. A timer or arm request can therefore
observe an incomplete pair and transiently report
`sensor_cloud_stamp_mismatch`.

## Selected Approach

Add a bounded, one-to-one timestamp pair tracker to the safety core and use it
in the UDP safety bridge.

The tracker maintains separate queues of unmatched pose and cloud stamps. When
either kind arrives, it searches the opposite queue for the nearest stamp
within the configured 20 ms tolerance. A match consumes both entries and
records the completion time of a new pair. Entries older than the existing
one-second retention window are removed, and both queues remain capacity
bounded.

One-to-one consumption is required: unrelated new messages must not repeatedly
reuse an old match and make the bridge appear healthy.

## Safety Decision

The bridge continues to check the existing independent freshness limits for
command, body pose, sensor pose, and front cloud.

Timestamp health is based on the most recently completed pair:

- Before both streams have produced data, report
  `sensor_cloud_stamp_missing`.
- After both streams exist but no valid pair has completed, report
  `sensor_cloud_stamp_mismatch`.
- After a pair completes, accept it only while its completion age is no greater
  than the stricter of the sensor-pose and front-cloud freshness limits.
- If new unmatched messages continue arriving, they do not refresh the
  completed-pair time. The bridge therefore rejects arming or disarms once the
  last valid pair becomes stale.

This changes only the real-capable UDP safety bridge. The legacy dry-run
`scan_m20_safety_bridge` does not control the UDP link and remains unchanged.

## Interfaces and Configuration

- No ROS topic, service, message type, or launch interface changes.
- Keep `max_sensor_cloud_stamp_delta: 0.02`.
- Keep `cloud_stamp_history_sec: 1.0` as the pair-tracker retention window.
- Keep all command, heartbeat, conflict, and freshness checks unchanged.
- Retain the adapter publication order `stamp`, then `pose`, then synced cloud;
  correctness no longer depends on that order.

## Testing

Add unit tests for:

- stamp-first and pose-first arrival producing one completed pair;
- values outside 20 ms not pairing;
- matched entries being consumed exactly once;
- expired unmatched entries being pruned;
- a fresh completed pair passing health checks;
- a missing, absent, or stale completed pair failing closed;
- continuing unmatched traffic not refreshing pair health.

Run the complete `m20_scan_bringup` Pytest suite, selected-package Colcon tests,
and rebuild `m20_scan_bringup` plus `m20_sensor_pose_adapter_cpp`.

## Runtime Validation

1. Explicitly disarm before restarting.
2. Restart the rebuilt adapter, planner, and safety bridges.
3. Observe sustained `ready` status with live pose and cloud inputs.
4. Confirm the pending command and UDP axes are zero.
5. Arm for 12 seconds while monitoring status and command output.
6. Require continuous `ARMED`, zero nonzero-command samples, no
   `sensor_cloud_stamp_mismatch`, and no robot motion.
7. Explicitly disarm and confirm the final state is `DISARMED`.

Any failed check leaves or returns the bridge to `DISARMED`; the safety bridge
must not be bypassed.
