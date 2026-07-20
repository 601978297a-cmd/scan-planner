# M20 Sensor Pose TF Threading Design

## Problem

`sensor_pose_adapter` currently uses `rclpy.spin(node)`, which runs a
single-threaded executor. The front-cloud subscription, retry timer, and TF
subscriptions therefore compete for the same executor thread. Under the
current real-sensor load, the adapter can time out clouds even though the
matching TF has already reached the ROS graph.

## Selected approach

Keep `tf2_ros.TransformListener(..., spin_thread=False)` and run the existing
node with `MultiThreadedExecutor(num_threads=2)`.

The transform listener already creates its subscriptions in a reentrant
callback group. Two executor threads allow those TF callbacks to run while the
node's default callback group handles the cloud subscription or retry timer.

## Scope

- Replace the implicit single-threaded `rclpy.spin(node)` call with an explicit
  two-thread `MultiThreadedExecutor`.
- Add a regression test for executor construction and shutdown behavior.
- Keep all topics, frames, QoS profiles, queue size, retry interval, TF lookup
  timestamp, and the 0.5-second timeout unchanged.
- Do not change SCAN-Planner control logic or UDP arming state.

## Lifecycle and failure handling

`main()` initializes ROS, creates the node and executor, adds the node, and
spins the executor. Cleanup shuts down the executor, destroys the node, and
shuts down ROS in that order.

TF lookup failures retain the current non-blocking retry behavior. This change
does not mask missing or invalid TF data by increasing the wait threshold.

## Verification

1. Run the adapter unit tests.
2. Build `m20_scan_bringup`.
3. Restart only the adapter with UDP remaining disarmed.
4. Observe the synchronized cloud output and adapter logs for at least 60
   seconds.
5. Confirm TF-wait timeouts no longer repeat under normal input and that
   synchronized front-cloud publication follows the available front-cloud
   rate.

