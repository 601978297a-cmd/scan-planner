# SCAN Planner world-frame alignment design

## Goal

Align the M20 SCAN Planner integration with the Super-LIO localization TF tree:

```text
world -> base_link_dog -> rslidar_front
```

## Changes

- Change the sensor pose adapter target frame from `map` to `world`.
- Change `grid_map.frame_id` from `map` to `world`.
- Change the M20 SCAN RViz fixed and grid reference frames from `map` to
  `world`.
- Update wiring tests to assert the new global frame.

`sliding_map`, `rslidar_front`, `/lio/robo/odom`, point-cloud topics, control
backends, safety gates, UDP output, and automatic arming remain unchanged.
No `map -> world` transform is added.

## Data flow

Super-LIO publishes robot odometry and `world -> base_link_dog`. The robot
model supplies `base_link_dog -> rslidar_front`. The sensor pose adapter looks
up `world -> rslidar_front` at each cloud timestamp and publishes the matched
sensor pose and cloud pair. SCAN Planner consumes all positions in `world`.

## Verification

- Run the M20 SCAN bringup tests.
- Build the affected packages.
- Confirm no M20 production configuration still uses `map` as its global
  frame.
