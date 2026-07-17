# M20 Lightning Body-Pose Stamp Adapter Design

Date: 2026-07-17

## Problem

The SCAN planner, closed-loop controller, sensor-pose adapter, and UDP safety
bridge all subscribe to `/lightning/odom`. The Python UDP bridge intermittently
reports `body_pose_stale` even while external measurements show Lightning odom
near 5 Hz. Repeated deserialization and DDS scheduling in the Python bridge add
work without providing value because the bridge only uses odom receipt time.

## Scope

Only the UDP safety bridge health input changes. SCAN planner and the
closed-loop controller continue consuming the complete `/lightning/odom`
message. Lightning and SCAN core algorithms are unchanged.

## Design

The existing `sensor_pose_from_odom_adapter` already receives every
`/lightning/odom` message. Its body-pose callback will publish the incoming
header as:

```text
/scan/body_pose_stamp
std_msgs/msg/Header
```

The UDP safety bridge will subscribe to `/scan/body_pose_stamp` instead of
directly subscribing to `/lightning/odom`. Receipt updates the existing
`body_pose_rx` freshness field. The header timestamp is retained for
diagnostics but is not used as a substitute for monotonic receipt-time
freshness.

The adapter continues to publish:

```text
/scan/sensor_pose
/scan/front_cloud_stamp
```

## Safety Behavior

- `body_pose_timeout` remains an independent wall-time freshness check.
- A stopped Lightning odom stream stops `/scan/body_pose_stamp` and still
  produces `body_pose_stale`.
- Sensor-pose and front-cloud freshness checks remain independent.
- Timestamp-history matching remains unchanged.
- The UDP bridge remains disabled and disarmed by default.
- `/NAV_CMD` remains disconnected.

The dry-run and initial navigation trial use 0.8-second body, sensor-pose, and
front-cloud freshness limits. These trial overrides do not change the checked-in
default configuration unless runtime evidence supports doing so.

## Tests

1. The adapter publishes a body header with the same stamp and frame as each
   received odom message.
2. The UDP bridge updates body freshness from a Header callback.
3. The UDP bridge no longer subscribes to `/lightning/odom`.
4. Missing or stale body headers still report the existing safety reasons.
5. All existing timestamp, protocol, and stop-sequence tests continue to pass.

## Runtime Acceptance

With Lightning and SCAN dry-run running:

- `/scan/body_pose_stamp` has one publisher and one UDP-bridge subscriber.
- The topic rate follows `/lightning/odom`.
- The UDP bridge can remain armed with zero command for at least 10 seconds
  without a false `body_pose_stale`.
- Stopping body-stamp updates still disarms the bridge.
- Before physical navigation, restore `enable_udp_output=false`,
  confirm `DISARMED`, and confirm zero `/NAV_CMD` publishers.
