# M20 Sensor Pose Publish Order Design

## Goal

Prevent the UDP safety bridge from transiently reporting
`sensor_cloud_stamp_mismatch` for a correctly matched front-cloud/pose pair.

## Cause

Both sensor pose adapter backends currently publish a matched pair in this
order:

1. `/scan/sensor_pose`
2. `/scan/front_cloud_stamp`
3. `/scan/front_cloud_synced`

The safety bridge can evaluate its cached inputs after the pose arrives but
before the corresponding cloud stamp arrives. During that interval, the newest
pose is compared with the previous cloud stamp and the bridge disarms with
`sensor_cloud_stamp_mismatch`.

## Change

Change only the first two publications in both adapter implementations:

1. `/scan/front_cloud_stamp`
2. `/scan/sensor_pose`
3. `/scan/front_cloud_synced`

The messages, timestamps, topics, QoS profiles, matching logic, safety
thresholds, and planner interfaces remain unchanged.

The UDP safety bridge retains a short history of cloud stamps. With the new
order, the transient stamp-with-old-pose state still matches the retained stamp
for the old pose; after the new pose arrives, it matches the already-received
new stamp. This removes the observed unmatched state without weakening the
bridge's timestamp tolerance or freshness checks.

## Testing

- Add a Python unit test that records cross-publisher events and asserts
  `stamp`, then `pose`, then `cloud`.
- Add a C++ source-wiring test that asserts the same publication order.
- Run the complete `m20_scan_bringup` Pytest and Colcon test suites.
- Rebuild `m20_scan_bringup` and `m20_sensor_pose_adapter_cpp`.

## Runtime Validation

1. Confirm the robot is `DISARMED`.
2. Restart the adapter, planner, and UDP safety bridge from the rebuilt
   workspace.
3. Confirm front-cloud stamps and sensor poses are live and matched.
4. Request a zero-velocity arm for 12 seconds.
5. Require no `sensor_cloud_stamp_mismatch`, no nonzero command output, and no
   robot motion.
6. Explicitly disarm and confirm the final state is `DISARMED`.

If any safety check fails, keep or return the bridge to `DISARMED`; do not
bypass the safety bridge.
