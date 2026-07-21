# SCAN Disarm Cancellation and Marker Frame Design

## Problem

The UDP safety bridge stops robot output when it disarms, but SCAN Planner and
the closed-loop controller retain the active goal and B-spline. When odometry
recovers, the controller can resume the old trajectory and publish a saturated
yaw command such as `angular.z = 0.20`. The arm gate prevents this stale command
from reaching the robot, but each new test currently requires restarting SCAN.

Planner visualization also mixes hard-coded `world` and `map` marker frames.
On M20, `grid_map.frame_id` is `map`; therefore `/optimal_list` is invisible in
an RViz session fixed to `map` when its marker is stamped as `world` and no
`map`/`world` transform exists. RViz also requests volatile durability for the
retained trajectory marker, so a display created after planning can miss it.

## Chosen Design

### Latched navigation enable state

The UDP safety bridge will publish `/scan/navigation_enabled` as
`std_msgs/msg/Bool` with reliable, transient-local, depth-one QoS.

- Publish `false` at bridge startup.
- Publish `true` only after all arm blockers pass and the bridge enters ARMED.
- Publish `false` immediately whenever the bridge begins any stop, including
  manual disarm, sensor freshness failures, heartbeat failures, command
  failures, and UDP errors.

Both C++ consumers default to disabled before receiving any message. This is a
fail-safe default if the bridge is absent or still starting.

### Planner cancellation

`SCANReplanFSM` will subscribe to `/scan/navigation_enabled` with matching QoS.
When it receives `false`, it will:

- clear the current and pending target flags;
- clear the active local-trajectory metadata needed by the execution FSM;
- reset replan/failure state associated with the old target;
- transition to `WAIT_TARGET` once odometry initialization is complete; and
- clear the old goal and trajectory visualization.

Goal and path callbacks will ignore new navigation requests while disabled.
After a later `true`, the planner remains in `WAIT_TARGET`; it will not resume
the cancelled target. The FSM records when enable was received and rejects
zero-stamped requests or requests whose header stamp predates that enable
event. A new, freshly stamped RViz goal is required.

### Controller cancellation and gating

`ClosedLoopController` will subscribe to `/scan/navigation_enabled` with
matching QoS. While disabled it will:

- reject incoming B-splines;
- clear `receive_traj_`, trajectory storage, duration, and execution time; and
- continue publishing zero `Twist` at the existing 100 Hz command rate.

When enabled, it records the enable time and accepts only B-splines whose
`start_time` is not older than that time. Receiving `false` again clears the
active trajectory immediately and keeps command output at zero. The latched
gate plus timestamp check removes the cross-topic ordering race that a one-shot
cancel message would have with an in-flight B-spline.

### Visualization frame

`PlanningVisualization` will receive the configured planner frame instead of
using hard-coded `world` or `map` strings. `SCANReplanFSM` will pass the existing
`grid_map.frame_id` value:

- M20 configuration: `map`;
- simulator configuration: `world`.

All planner markers, including `/optimal_list` and `/goal_point`, will use this
frame consistently. The M20 RViz `/optimal_list` Marker display will request
transient-local durability so it receives the latest retained trajectory even
when the display starts after planning.

The temporary runtime identity transform between `map` and `world` is not part
of the final design and will be stopped after deployment.

## Alternatives Rejected

1. **One-shot cancel topic:** smaller change, but an in-flight B-spline can
   arrive after cancel and reactivate the controller.
2. **Sequential planner/controller reset services:** deterministic when every
   service succeeds, but adds asynchronous orchestration and failure handling
   to the safety bridge. A latched enable gate is simpler and remains safe when
   nodes start in any order.
3. **Remove the arm gate:** rejected because a retained goal or recovered
   localization could move the real robot without a new operator decision.

## Failure Behaviour

- Missing bridge or missing navigation state keeps Planner and controller
  disabled.
- Any safety stop publishes disabled before or while zero UDP stop packets are
  sent.
- Repeated disabled messages are idempotent.
- A goal sent while disabled is ignored, rather than queued for later motion.
- A queued goal or B-spline stamped before the latest enable is ignored.
- Re-arming never restores a cancelled target; the operator must send a new
  goal.

## Verification

Automated checks will verify:

- bridge startup, successful arm, manual disarm, and automatic stop publish the
  expected navigation state;
- controller ignores trajectories while disabled, clears an active trajectory
  on disable, and continuously publishes zero afterward;
- Planner drops its target and returns to `WAIT_TARGET` on disable;
- planner visualization uses the configured frame;
- M20 RViz requests transient-local durability for `/optimal_list`;
- focused tests, the full test suite, and the affected packages build cleanly.

Real-robot verification will use the existing low limits (`0.05 m/s` forward,
`0.20 rad/s` yaw): arm at zero, send a short goal, disarm during execution,
confirm immediate zero output, re-arm, and confirm that no motion occurs until
a new goal is sent. `/optimal_list` must report `frame_id: map` and be visible in
RViz fixed to `map`.
