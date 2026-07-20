# Reliable SCAN Safety Signal QoS Design

Date: 2026-07-20

## Goal

Prevent the M20 safety bridges from falsely disarming because the lightweight
`/scan/cmd_vel_debug` and `/scan/front_cloud_stamp` messages are missed under
the current DDS load.

## Scope

Make only these two safety-signal paths end-to-end reliable:

- `/scan/cmd_vel_debug`
- `/scan/front_cloud_stamp`

Do not change lidar drivers, Fast DDS configuration, motion limits, freshness
timeouts, stamp matching tolerances, UDP mapping, or arming rules.

## Design

The closed-loop controller already publishes `/scan/cmd_vel_debug` with the
default reliable ROS 2 publisher QoS, so its publisher remains unchanged.
Both the dry-run safety bridge and UDP safety bridge will subscribe to this
topic with reliable, volatile, keep-last depth 5 QoS.

The C++ sensor pose adapter and the Python fallback will publish
`/scan/front_cloud_stamp` with reliable, volatile, keep-last depth 5 QoS.
Both safety bridges will subscribe with the matching profile.

The large synchronized point cloud and paired pose QoS introduced by commit
`2e74666` remains unchanged.

## Failure Handling

All existing safety behavior remains active. A genuinely stale command,
position, sensor pose, cloud stamp, heartbeat error, conflicting controller,
or stamp mismatch still rejects arming or immediately starts the zero-command
stop sequence. Recovery never rearms automatically.

## Tests and Runtime Validation

Add static wiring tests that verify:

- both sensor pose adapter implementations publish the cloud stamp reliably;
- both safety bridges use reliable QoS for the command and cloud stamp
  subscriptions;
- the command publisher remains reliable.

Build the affected packages and run the existing test suites. Restart only the
SCAN launch group and UDP safety bridge while the UDP bridge is disarmed.
Confirm the topic endpoint QoS with `ros2 topic info -v`.

For physical validation:

1. Confirm localization, TF, sensor pose, cloud stamp, command, and heartbeat
   are fresh.
2. Confirm the pending command and UDP axes are zero.
3. Arm without a navigation goal.
4. Hold ARMED for at least 10 seconds with zero axes and no
   `command_stale`, `front_cloud_stale`, or stamp mismatch.
5. Manually disarm and confirm the zero-command stop sequence completes.

