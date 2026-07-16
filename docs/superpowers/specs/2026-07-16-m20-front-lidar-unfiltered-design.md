# M20 Front LiDAR Unfiltered SCAN Dry Run

## Baseline and rollback

The fixed-height-filter dry-run is preserved at commit `47b76c4` and branch
`backup/m20-fixed-height-dryrun-20260716`.

## Data flow

- `/lightning/odom` supplies the M20 body pose in `map`.
- `/rslidar_points` supplies raw front-LiDAR points in the front sensor coordinate system.
- `sensor_pose_adapter` uses each front-cloud timestamp to publish `map -> rslidar_front`
  as `/scan/sensor_pose`.
- SCAN receives the raw cloud with `grid_map.cloud_is_world=false` and transforms points
  internally using `/scan/sensor_pose`.
- `grid_map.need_extrinsic=false` prevents application of the hard-coded Go2 extrinsic;
  the M20 extrinsic comes from `base_link -> rslidar_front` TF.

No ground or fixed-height filter is started in this version.

## Safety boundary

The closed-loop controller remains remapped to `/scan/cmd_vel_debug`. The launch file
must not publish `/NAV_CMD`, `/GAIT`, or the real `/cmd_vel` topic.

## Verification

- Build `m20_scan_bringup` successfully.
- Confirm the launch contains no `cloud_filter_adapter` node.
- Confirm SCAN cloud input is `/rslidar_points`.
- Confirm sensor pose source is `rslidar_front` and follows `/rslidar_points` timestamps.
- Confirm `cloud_is_world=false` and `need_extrinsic=false`.
- Confirm controller output remains `/scan/cmd_vel_debug`.
