# SCAN Cloud and Sensor Pose Synchronization

## Goal

Feed SCAN only front-LiDAR clouds that have an exact
`map -> rslidar_front` transform at the cloud timestamp. Keep Super-LIO and
SCAN core code unchanged, and keep real motion output disarmed during
validation.

## Design

`sensor_pose_adapter` stores incoming `/rslidar_points_front` messages in a
bounded FIFO. A 20 ms timer performs non-blocking TF lookups at each cloud's
exact timestamp. Once available, the adapter publishes:

1. `/scan/sensor_pose`
2. `/scan/front_cloud_stamp`
3. `/scan/front_cloud_synced`

All three outputs use the original cloud timestamp. SCAN subscribes to the
synchronized cloud topic rather than the raw front cloud.

Clouds waiting more than 0.5 seconds or exceeding the 16-message queue are
dropped. Raw clouds are never passed to SCAN without a matching sensor pose,
and latest-time TF is never substituted for the requested timestamp.

## Verification

- Unit tests cover delayed TF availability, matching timestamps, and timeout
  drops.
- Runtime checks compare raw and synchronized cloud rates and timestamps.
- `/scan/sensor_pose` must remain fresh and match the synchronized cloud stamp.
- UDP stays disarmed and `/NAV_CMD` keeps zero publishers during validation.
