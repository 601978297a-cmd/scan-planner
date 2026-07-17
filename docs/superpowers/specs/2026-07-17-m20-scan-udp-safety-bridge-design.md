# M20 SCAN UDP Safety Bridge Design

Date: 2026-07-17

## Goal

Add a guarded UDP control backend for SCAN-Planner in the
`m20_scan_bringup` package. The backend converts the closed-loop controller's
`Twist` output into the M20 UDP axis protocol already used by
`/home/nvidia/hqs/lynx_m20_base`.

SCAN-Planner remains responsible for localization, mapping, planning, and
closed-loop velocity generation. The M20 built-in navigation task interface is
not used.

## Scope

The change will:

- add an independent `scan_m20_udp_safety_bridge` node;
- keep the existing `/NAV_CMD` safety bridge unchanged;
- add a launch argument that selects exactly one control backend;
- reuse the existing input freshness checks and safety-state concepts;
- implement the HQS UDP packet format, heartbeat, axis mapping, arming,
  conflict checks, and explicit zero-command shutdown;
- allow forward motion and yaw only;
- remain disabled and disarmed by default.

The change will not:

- publish `/NAV_CMD`, `/GAIT`, or `/MOTION_STATE` from the UDP bridge;
- use the M20 built-in navigation task API;
- allow reverse or lateral motion in the first version;
- automatically re-arm after any stop or fault;
- enable physical motion during implementation or automated testing.

## Architecture

Inputs:

- `/scan/cmd_vel_debug` (`geometry_msgs/msg/Twist`)
- `/lightning/odom` (`nav_msgs/msg/Odometry`)
- `/scan/sensor_pose` (`nav_msgs/msg/Odometry`)
- `/scan/front_cloud_stamp` (`std_msgs/msg/Header`)

The existing sensor-pose adapter publishes `/scan/front_cloud_stamp` for every
received `/rslidar_points_front` message, before pose matching. This preserves
an independent cloud-freshness and timestamp check without making the safety
bridge deserialize another approximately 1 MB point cloud. Directly subscribing
the Python safety bridge to the full cloud reduced measured delivery rate and
added about one CPU core of avoidable work.

Outputs:

- `/scan/udp_axis_preview` (`geometry_msgs/msg/Twist`)
- `/scan/udp_safety_status` (`diagnostic_msgs/msg/DiagnosticArray`)

Service:

- `/scan/arm_udp_control` (`std_srvs/srv/SetBool`)

UDP endpoint:

- target: `10.21.31.103:30000`
- heartbeat: `Type=100`, `Command=100`
- motion axis: `Type=2`, `Command=21`
- packet header and JSON layout match `hqs/lynx_m20_base`

The UDP bridge uses heartbeat acknowledgements for robot-link liveness. It does
not depend on the external `drdds` workspace.

## Backend Selection

`m20_scan_dry_run.launch.py` gains a `control_backend` argument:

- `nav_cmd`: starts the existing guarded `/NAV_CMD` bridge;
- `udp`: starts the new UDP bridge and does not start the `/NAV_CMD` bridge.

The default remains `nav_cmd`, preserving current behavior. Both backend
configuration files keep physical output disabled by default.

## State And Arming

The UDP bridge uses three states:

- `DISARMED`: no motion-axis UDP packets are sent;
- `ARMED`: guarded UDP motion packets are sent;
- `STOPPING`: zero-axis packets are sent before returning to `DISARMED`.

Arming is rejected unless:

- `enable_udp_output` is true;
- the bridge is `DISARMED`;
- command, body pose, sensor pose, and front cloud inputs are fresh;
- sensor-pose and cloud timestamps match within the configured tolerance;
- a recent heartbeat acknowledgement with `ErrorCode=0` exists;
- no known competing local controller is running;
- no non-finite command value is present.

Known competing processes include `key_test` and `m20_udp_drive_test.py`.
Process detection is an interlock, not a complete guarantee against every
possible external UDP sender. Operator procedure must still ensure a single
controller.

Any fault while armed starts the zero-command sequence. Fault recovery never
re-arms the bridge.

## Command Mapping

Default physical limits:

- maximum forward speed: `0.05 m/s`;
- maximum yaw speed: `0.20 rad/s`;
- reverse command: forced to zero;
- lateral command: forced to zero.

Default UDP-axis limits:

- maximum X axis: `0.50`;
- yaw dead-zone floor: `0.50`;
- maximum yaw axis: `1.00`.

Forward mapping:

```text
vx <= 0: udp_x = 0
vx > 0:  udp_x = 0.50 * clamp(vx / 0.05, 0, 1)
```

Yaw mapping:

```text
near zero: udp_yaw = 0
otherwise:
  udp_yaw = sign(wz) *
            (0.50 + 0.50 * clamp(abs(wz) / 0.20, 0, 1))
```

Yaw start/stop hysteresis prevents sign chatter around zero. Physical command
slew limits are applied before UDP mapping. All non-finite values map to zero.

These values are parameters and will be refined only through supervised
physical calibration.

## Timing And Shutdown

- normal motion command rate: `10 Hz`, matching the HQS Web controller;
- heartbeat rate: `1 Hz`;
- timer base rate: `20 Hz`;
- stop sequence: 20 zero-axis packets at `20 Hz`;
- node shutdown: best-effort zero sequence before socket closure.

With physical output enabled but the bridge disarmed, heartbeat packets may be
sent for link diagnostics, but motion-axis packets are not sent.

## Diagnostics

The diagnostic output includes:

- bridge state;
- physical-output enable flag;
- heartbeat freshness and last error code;
- current health blockers;
- conflicting process names;
- limited physical command;
- mapped UDP X and Yaw values;
- zero packets remaining during `STOPPING`;
- last stop reason.

The preview topic contains normalized UDP axes:

- `linear.x`: UDP X;
- `linear.y`: always zero;
- `angular.z`: UDP Yaw.

## Verification

Automated tests cover:

- packet header, payload length, and JSON fields;
- heartbeat acknowledgement parsing;
- forward and yaw mapping, limits, dead zone, and hysteresis;
- rejection of reverse, lateral, NaN, and infinity;
- arming gates and conflict detection;
- timeout-to-stop transitions;
- exactly 20 stop packets;
- loopback UDP integration with a local mock server;
- launch selection ensuring only one backend starts.

Physical verification is staged:

1. Start with `enable_udp_output=false`; verify preview only.
2. Set `enable_udp_output=true` but remain `DISARMED`; verify heartbeat only.
3. Stop competing controllers, arm with no goal, and verify zero axis.
4. Set a straight goal approximately `0.3 m` ahead with `max_vx=0.05 m/s`.
5. Manually disarm and induce a sensor timeout; verify immediate stop and no
   automatic re-arm.
6. Confirm no new publishers exist for `/NAV_CMD`, `/GAIT`, or
   `/MOTION_STATE`.

Steps 3-5 require a clear area, an operator holding the physical emergency
control, and explicit approval immediately before the test.
