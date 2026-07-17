# M20 SCAN Super-LIO Backend Design

## Scope

Switch only the M20 SCAN integration from Lightning localization to the
already-running Super-LIO relocation output. Do not modify Super-LIO source,
configuration, launch files, maps, or runtime parameters.

The guarded control behavior remains unchanged:

- SCAN controller output stays on `/scan/cmd_vel_debug`.
- UDP output stays disabled by default and requires manual arming.
- No node added by this change publishes `/NAV_CMD`, `/GAIT`, or
  `/MOTION_STATE`.

## Data Flow

- Body pose: `/lio/robo/odom` (`nav_msgs/msg/Odometry`, frame `world`)
- Front obstacle cloud: `/rslidar_points_front`
  (`sensor_msgs/msg/PointCloud2`, frame `rslidar_front`)
- Front sensor pose: combine `/lio/robo/odom` with the existing
  `base_link -> rslidar_front` static transform, then publish
  `/scan/sensor_pose` in `world`
- Planner body pose: `/lio/robo/odom`
- Controller body pose: `/lio/robo/odom`
- Planner global frame: `world`
- Planner velocity output: `/scan/cmd_vel_debug`

The Super-LIO localization input remains `/rslidar_points` plus `/IMU`.
The SCAN obstacle input remains the independent front cloud
`/rslidar_points_front`.

## Changes

1. Update `m20_scan_dry_run.launch.py`:
   - Remove the visualization-only `map -> world` static transform.
   - Set the sensor-pose adapter target frame to `world`.
   - Change every SCAN body-pose remap from `/lightning/odom` to
     `/lio/robo/odom`.
2. Update `m20_scan_planner.yaml`:
   - Set `grid_map.frame_id` to `world`.
   - Keep `grid_map.cloud_is_world: false`.
   - Keep `grid_map.need_extrinsic: false`.
3. Update the M20 RViz configuration:
   - Use `world` as the fixed frame.
   - Preserve the existing front-cloud, occupancy, path, and safety displays.
4. Update bringup documentation and tests that encode the old Lightning topic
   or `map` frame.

No compatibility adapter or changes to Super-LIO will be introduced.

## Failure Handling

- If `/lio/robo/odom` is stale, the existing safety bridge disarms.
- If the front cloud or derived sensor pose is stale or their timestamps do
  not match, the existing safety bridge disarms.
- A missing `base_link -> rslidar_front` transform prevents sensor-pose
  publication; no zero or fabricated pose is published.

## Verification

1. Build `m20_scan_bringup` and run its tests.
2. Launch in dry-run mode with UDP output disabled.
3. Verify all SCAN consumers use `/lio/robo/odom` and none use
   `/lightning/odom`.
4. Verify `/scan/sensor_pose.header.frame_id`, occupancy, and planned paths use
   `world`.
5. Verify `/NAV_CMD` has zero publishers.
6. Verify Super-LIO files and git status are unchanged.

