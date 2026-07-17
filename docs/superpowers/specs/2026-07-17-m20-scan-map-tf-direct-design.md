# M20 SCAN Direct Map TF Design

## Current Contract

Super-LIO now publishes:

- `/lio/robo/odom` with `header.frame_id=map` and
  `child_frame_id=base_link`
- dynamic TF `map -> base_link`

The robot TF tree publishes the verified static transform
`base_link -> rslidar_front`. Therefore tf2 can resolve
`map -> rslidar_front` directly.

This change modifies only the SCAN integration. Super-LIO source,
configuration, launch files, and runtime parameters are not modified.

## SCAN Data Flow

- `body_pose <- /lio/robo/odom`
- `cloud <- /rslidar_points_front`
- For each front-cloud timestamp, query TF `map -> rslidar_front`
- Publish that transform as `/scan/sensor_pose`
  (`nav_msgs/msg/Odometry`)
- Set the SCAN grid map and RViz fixed frame to `map`

The cloud remains in `rslidar_front`; `cloud_is_world` remains false and
`need_extrinsic` remains false.

## Adapter Scope

Use the existing TF-based `sensor_pose_adapter` only as a SCAN interface
converter. It does not inspect, filter, copy, or transform point fields. It
reads the PointCloud2 header timestamp, queries tf2, and publishes:

- `/scan/sensor_pose`
- `/scan/front_cloud_stamp`

The previous odometry-composition adapter is no longer launched. The fixed
height cloud-filter adapter is removed from package entry points because raw
front lidar is connected directly to SCAN.

## Safety Inputs

The UDP safety bridge independently subscribes to full
`/lio/robo/odom` for body-pose freshness. It continues to subscribe to the
lightweight front-cloud stamp and `/scan/sensor_pose`.

No safety timeout is relaxed. UDP output remains manually armed, defaults to
disabled, and is kept `DISARMED` during verification.

## Verification

1. Confirm all active SCAN frames are `map`.
2. Confirm the TF adapter publishes `map -> rslidar_front` at front-cloud
   timestamps without extrapolation errors.
3. Confirm SCAN consumes `/lio/robo/odom`, `/rslidar_points_front`, and
   `/scan/sensor_pose`.
4. Confirm occupancy publishes in `map`.
5. Confirm the UDP bridge remains `DISARMED`, heartbeat error is zero, and
   `/NAV_CMD` has zero publishers.
6. Send only a dry-run goal and confirm B-spline plus
   `/scan/cmd_vel_debug`; do not arm hardware output.

