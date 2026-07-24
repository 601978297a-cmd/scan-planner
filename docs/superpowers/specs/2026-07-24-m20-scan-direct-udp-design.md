# M20 SCAN Planner Direct UDP Control Design

## Goal

Replace the current `/NAV_CMD` execution backend used by the combined M20
localization and navigation launch with a direct UDP bridge based on
`Super-LIO-DEV/navigation_bringup/cmd_vel_to_udp.cpp`.

The planner and controller algorithms remain unchanged. Only the final command
transport changes.

## Selected Design

The direct UDP node subscribes to the existing controller output topic
`/scan/cmd_vel_debug`. It does not use the NAV_CMD safety bridge, gait checks,
motion-state checks, sensor freshness gates, an arm service, or UDP heartbeat
validation.

For every incoming `geometry_msgs/msg/Twist`, it calculates:

- `X = clamp(linear.x * 3.0, -1.0, 1.0)`
- `Y = clamp(linear.y * 3.0, -1.0, 1.0)`
- `Yaw = clamp(angular.z * 2.0, -3.0, 3.0)`
- `Z`, `Roll`, and `Pitch` are always zero.

The command is encoded as the M20 `PatrolDevice` JSON motion command
(`Type=2`, `Command=21`) with the same 16-byte binary header used by
Super-LIO-DEV, then sent to `10.21.31.103:30000`.

## Runtime Behaviour

- Starting the node sends the M20 stand command (`Command=22`,
  `MotionParam=1`) once.
- Each Twist callback sends one UDP motion packet immediately. The UDP command
  frequency therefore follows the ScanPlanner controller output frequency.
- If no Twist arrives for 500 ms, the node sends a zero motion command once.
- Shutting down the node sends a zero motion command before closing the socket.
- The combined localization and navigation launch selects this direct UDP node
  by default and does not start the NAV_CMD backend at the same time.
- RViz remains a separate launch and is unchanged.

## Scope

The implementation reuses the existing UDP packet builder where practical and
adds only the direct command node and launch wiring needed for this backend.
Planning, collision checking, trajectory generation, controller gains,
localization, TF, lidar processing, and RViz configuration are out of scope.

## Safety Consequences

This design intentionally removes the ScanPlanner safety bridge from the final
command path. A valid Twist is forwarded without checking gait, robot motion
state, localization freshness, lidar freshness, navigation-enabled state, or
the presence of another motion publisher. Physical testing therefore requires
clear space and an independent stop method.

The 500 ms command timeout and shutdown zero packet are the only automatic stop
protections in this direct backend.

## Verification

Verification will cover:

1. Unit tests for scaling, clamping, packet contents, timeout zero output, and
   shutdown zero output without contacting the robot.
2. Launch-wiring tests proving the combined launch starts only the direct UDP
   backend and points it at `/scan/cmd_vel_debug`.
3. Package tests and a targeted colcon build on SSH 174.
4. A dry launch inspection that confirms the configured target, topic, scales,
   and timeout. No physical movement command will be sent during automated
   verification.

## Rollback

The pre-change state is preserved at branch
`backup/m20-before-direct-udp-20260724`, commit `025cfd7`.
