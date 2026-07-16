# M20 SCAN Navigation Safety Bridge Design

## Objective

Connect SCAN-Planner's closed-loop debug velocity to an independently guarded
M20 command bridge. The first acceptance stage remains a dry run: it publishes
only a limited command preview and never creates a `/NAV_CMD` publisher.

This work does not publish `/GAIT`, does not change the robot gait, and does not
run a live motion calibration.

## Front Lidar Input

SCAN uses only the independently published front lidar cloud:

- cloud: `/rslidar_points_front`, frame `rslidar_front`
- sensor pose: `/scan/sensor_pose`, frame transform `map -> rslidar_front`
- body pose: `/lightning/odom`
- `grid_map.cloud_is_world: false`
- `grid_map.need_extrinsic: false`

The merged `/rslidar_points` cloud remains available to Lightning and other
consumers but is not used with the single front-lidar ray origin.

## Cloud and Pose Synchronization

`sensor_pose_from_odom_adapter` continues to stamp each accepted sensor pose
with the source cloud timestamp. SCAN's lidar input changes from two independent
callbacks to an exact-time `message_filters` pair for `cloud` and
`sensor_pose`. A cloud without a same-stamp sensor pose is not inserted into the
occupancy map. This change affects input synchronization only, not raycasting,
occupancy, search, or trajectory optimization algorithms.

## Bridge Interfaces

Inputs:

- `/scan/cmd_vel_debug` (`geometry_msgs/msg/Twist`)
- `/lightning/odom` (`nav_msgs/msg/Odometry`)
- `/scan/sensor_pose` (`nav_msgs/msg/Odometry`)
- `/rslidar_points_front` (`sensor_msgs/msg/PointCloud2`)
- `/MOTION_INFO` (`drdds/msg/MotionInfo`) only when real-output capability is enabled

Outputs:

- `/scan/nav_cmd_preview` (`geometry_msgs/msg/Twist`), always safe to inspect
- `/scan/nav_safety_status` (`diagnostic_msgs/msg/DiagnosticArray`)
- `/NAV_CMD` (`drdds/msg/NavCmd`), created only after all real-output gates pass

Control service:

- `/scan/arm_nav_cmd` (`std_srvs/srv/SetBool`)

## Command Mapping

The bridge maps the limited preview to M20 fields as follows:

- `Twist.linear.x` -> `NavCmd.data.x_vel`
- lateral velocity is forced to zero
- reverse velocity is forced to zero
- `Twist.angular.z` -> `NavCmd.data.yaw_vel`

Initial limits are 0.05 m/s forward and 0.30 rad/s yaw. Linear and yaw
acceleration limits are applied at the 20 Hz output rate. The configurable yaw
dead-zone mapping is:

- input magnitude at or below `yaw_zero_epsilon` -> zero
- other nonzero input -> at least `min_yaw_cmd`, preserving sign
- result -> capped at `max_yaw`

`min_yaw_cmd` remains zero until a supervised calibration determines the real
robot's threshold.

## Safety State Machine

The bridge has three states:

1. `DISARMED`: preview and diagnostics are active; no `/NAV_CMD` publisher exists.
2. `ARMED`: all gates passed and the bridge is the only `/NAV_CMD` publisher.
3. `STOPPING`: zero commands are published for five 20 Hz cycles, then the
   `/NAV_CMD` publisher is destroyed and the state returns to `DISARMED`.

Arming requires all of the following:

- launch parameter `enable_m20_output` is true
- launch parameter `yaw_deadzone_calibrated` is true
- a positive manual `/scan/arm_nav_cmd` request
- no existing `/NAV_CMD` publisher
- fresh command, body pose, sensor pose, front cloud, and M20 motion information
- sensor-pose and front-cloud header timestamps differ by no more than 0.02 s

Default freshness limits are 0.20 s for commands, 0.50 s for body and sensor
poses, 0.30 s for the front cloud, and 0.50 s for motion information.

While armed, any stale input, cloud/pose timestamp mismatch, additional
`/NAV_CMD` publisher, or internal exception immediately enters `STOPPING`.
Recovery never rearms automatically. Shutdown also publishes the zero sequence
when a real publisher exists.

## Runtime Dependency Isolation

The preview path does not require `drdds`. The bridge imports `drdds` and creates
the M20 motion-information subscription only when `enable_m20_output=true`. If
the type support is unavailable, arming is rejected and `/NAV_CMD` is not
created.

## Launch Defaults

The normal M20 dry-run launch starts the bridge with:

- `enable_m20_output: false`
- `yaw_deadzone_calibrated: false`
- `min_yaw_cmd: 0.0`

Therefore starting the normal launch cannot create a `/NAV_CMD` publisher.

## Verification

Automated tests cover limiting, reverse/lateral rejection, dead-zone mapping,
acceleration limiting, freshness checks, timestamp matching, arm rejection, and
stop sequencing.

Dry-run runtime acceptance requires:

1. SCAN subscribes to `/rslidar_points_front` and paired sensor poses.
2. `/scan/nav_cmd_preview` is limited to the configured bounds.
3. stale inputs force the preview to zero and produce a diagnostic reason.
4. `/NAV_CMD` has zero publishers throughout the test.
5. `/GAIT` has zero publishers from this bringup.

Live calibration and live navigation are separate supervised acceptance steps.
They require a person beside the robot, a working physical stop mechanism, and
an obstacle-free test area.
