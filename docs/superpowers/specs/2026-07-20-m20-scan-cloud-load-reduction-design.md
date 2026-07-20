# M20 SCAN Cloud Load Reduction Design

## Problem

The live SCAN stack has two Python nodes subscribing to the complete
`/rslidar_points_front` `PointCloud2` stream:

- `sensor_pose_adapter` needs the cloud to match it with TF and republish the
  synchronized planning input.
- `scan_m20_safety_bridge` only reads the message timestamp and receipt time.

Deserializing the same large cloud in both nodes wastes CPU. The live Foxglove
bridge also permits raw, merged, and synchronized point-cloud topics through
broad regular expressions, making it easy for a dashboard to add several
large-data subscriptions during real-robot validation.

## Selected approach

Use the lightweight header already published by `sensor_pose_adapter` as the
dry-run safety input, and restrict the live Foxglove bridge to the single cloud
that SCAN-Planner actually consumes.

The data flow becomes:

1. `sensor_pose_adapter` receives `/rslidar_points_front`.
2. It publishes the matched cloud on `/scan/front_cloud_synced` and its header
   on `/scan/front_cloud_stamp`.
3. `scan_m20_safety_bridge` receives `/scan/front_cloud_stamp` as
   `std_msgs/Header` and applies the existing freshness and stamp checks.
4. Foxglove port 8765 may expose `/scan/front_cloud_synced`, but no raw or
   merged RoboSense cloud.

## Code and configuration changes

### SCAN-Planner repository

- Replace the dry-run safety bridge's `PointCloud2` subscription with a
  `Header` subscription.
- Rename its parameter from `front_cloud_topic` to
  `front_cloud_stamp_topic`.
- Set the default and YAML value to `/scan/front_cloud_stamp`.
- Preserve the existing `front_cloud_timeout`, stamp matching, arming logic,
  topics unrelated to the cloud input, and control behavior.
- Add a regression test that verifies the lightweight subscription type and
  topic.

### Live Foxglove bridge

- Back up `/home/nvidia/run_m20_foxglove_bridge.sh` before editing because it
  is outside the SCAN-Planner Git repository.
- Remove `/rslidar_points.*` from the port-8765 whitelist.
- Replace broad `/scan/.*` access with this explicit list:
  `/scan/front_cloud_synced`, `/scan/front_cloud_stamp`,
  `/scan/sensor_pose`, `/scan/cmd_vel_debug`, `/scan/nav_cmd_preview`,
  `/scan/nav_safety_status`, `/scan/udp_axis_preview`, and
  `/scan/udp_safety_status`.
- Leave the port-8766 Huaxin playback bridge unchanged because it runs in ROS
  domain 42 and serves a separate workflow.

## Safety and failure behavior

The UDP bridge remains disarmed throughout restart and validation. Changing
the safety bridge payload type does not relax freshness thresholds: failure
to receive `/scan/front_cloud_stamp` for 0.30 seconds still produces
`front_cloud_stale`.

If the stamp publisher is absent or incompatible, the bridge receives no
headers and fails closed through the existing timeout path.

## Verification

1. Confirm the new regression test fails before the implementation.
2. Run all `m20_scan_bringup` tests and build the package.
3. Restart the SCAN launch group with UDP still `DISARMED`.
4. Restart only `m20-foxglove-bridge.service`; do not restart the domain-42
   playback bridge.
5. Confirm the dry-run safety bridge subscribes to
   `/scan/front_cloud_stamp` as `std_msgs/Header` and no longer subscribes to
   `/rslidar_points_front`.
6. Confirm the port-8765 command exposes only
   `/scan/front_cloud_synced` among the large SCAN/RoboSense clouds.
7. Measure raw and synchronized cloud rates, safety status, and the CPU usage
   of `sensor_pose_adapter` and `scan_m20_safety_bridge`.

Success requires unchanged safety behavior, no TF-drop warnings, lower
dry-run safety-bridge CPU usage, and no new synchronized-cloud loss relative
to the raw front-cloud input.
