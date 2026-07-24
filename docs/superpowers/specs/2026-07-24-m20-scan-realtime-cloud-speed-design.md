# M20 SCAN real-time cloud and speed design

## Goal

Reduce stale-map and stale-planning latency while raising the commanded forward
speed ceiling to 0.5 m/s. Preserve timestamp pairing between each lidar cloud
and its sensor pose.

## Data path

The sensor pose adapter continues to publish a cloud and pose with identical
timestamps. Its pending TF queue retains only the newest cloud. The planner
uses a reliable, shallow exact-time pair queue so old pairs cannot accumulate.

Before ray casting, the planner applies a 0.10 m PCL voxel-grid filter. Invalid
points continue to be rejected by the existing checks. The filtered cloud,
rather than the roughly 52,000-point raw cloud, drives occupancy updates.

Internal occupancy update timers remain unchanged. This work does not lower the
map update rate and does not introduce planner multithreading. RViz publication
behavior is outside this change.

## Speed mapping

Set both planner velocity limits to 0.5 m/s:

- `manager.max_vel: 0.5`
- `optimization.max_vel: 0.5`

Set the closed-loop forward clamp to `max_vx: 0.5`. Keep the existing lateral,
yaw, acceleration, collision, and emergency limits unchanged.

Set direct UDP `scale_x: 2.0`, giving:

- 0.25 m/s command -> UDP X 0.5
- 0.50 m/s command -> UDP X 1.0

UDP X is a normalized command. Physical speed at X 1.0 must be confirmed on the
robot.

## Verification

- Unit-test voxel filtering and direct UDP scaling.
- Build the affected ROS packages.
- Confirm the synchronized input queue is shallow/latest-only.
- Restart only with operator approval because the robot control stack is live.
- Measure planner CPU, filtered point count, occupancy output rate, and planning
  response latency.
- Perform the first 0.5 m/s motion test in a clear area with emergency stop
  available.
