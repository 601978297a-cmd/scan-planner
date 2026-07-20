# C++ Sensor Pose Adapter Design

## Goal

Replace the active Python `sensor_pose_adapter` with a C++ implementation that can receive TF, receive front point clouds, and match/publish queued clouds concurrently. Preserve all existing ROS interfaces and retain the Python implementation as an explicit fallback.

## Scope

The change is limited to the sensor pose adapter and its launch wiring:

- Add a new `ament_cmake` package named `m20_sensor_pose_adapter_cpp`.
- Add a C++ executable named `sensor_pose_adapter_cpp`.
- Update `m20_scan_dry_run.launch.py` to select the C++ adapter by default.
- Keep the current Python source and console entry point unchanged.
- Do not change SCAN-Planner, either safety bridge, SuperLIO, lidar drivers, TF publishers, command output, or UDP arming state.

## ROS Interface Compatibility

The C++ node keeps the node name `sensor_pose_adapter` and the existing parameters and defaults:

- `target_frame`: `map`
- `source_frame`: `rslidar_front`
- `cloud_topic`: `/rslidar_points_front`
- `output_topic`: `/scan/sensor_pose`
- `cloud_stamp_topic`: `/scan/front_cloud_stamp`
- `synced_cloud_topic`: `/scan/front_cloud_synced`
- `max_tf_wait_sec`: `0.5`
- `retry_period_sec`: `0.02`
- `max_queue_size`: `16`

It publishes the same message types:

- `nav_msgs/msg/Odometry` on `/scan/sensor_pose`
- `std_msgs/msg/Header` on `/scan/front_cloud_stamp`
- `sensor_msgs/msg/PointCloud2` on `/scan/front_cloud_synced`

All sensor topics use sensor-data QoS, matching the Python implementation.

## Concurrency Architecture

Three execution paths run independently:

1. **TF ingestion:** `tf2_ros::TransformListener` runs with its dedicated spin thread and continuously fills `tf2_ros::Buffer`.
2. **Cloud ingestion:** the point-cloud subscription uses its own mutually exclusive callback group. Its callback only timestamps and enqueues the shared `PointCloud2` message.
3. **Matching and publishing:** the retry timer uses a separate mutually exclusive callback group. It checks queued clouds against the TF buffer and publishes matched outputs.

The node itself runs in a two-thread `MultiThreadedExecutor`. Together with the TransformListener thread, TF ingestion, cloud enqueue, and matching/publishing can proceed concurrently.

## Queue and Locking

A mutex protects only the pending-cloud deque.

- The cloud callback holds the mutex only while enforcing the queue limit and appending.
- The timer holds the mutex only while inspecting or removing the current queue entry.
- TF lookup, warning output, and ROS publication happen without holding the queu mutex.
- Removal verifies that the same queud entry is still at the front before popping it.

The queue preserves FIFO behavior and existing safety semantics. When the queue is full, the oldest cloud is removed and warned. When the oldest cloud waits longer than `max_tf_wait_sec`, it is removed and warned. A TF lookup miss leaves the cloud queued for the next timer tick.

Point clouds are stored and passed as shared message pointers to avoid copying their large data buffers inside the adapter.

## Launch and Fallback

Add launch argument `sensor_pose_adapter_backend` with choices `cpp` and `python`, defaulting to `cpp`.

- `cpp` starts package `m20_sensor_pose_adapter_cpp`, executable `sensor_pose_adapter_cpp`.
- `python` starts the current package `m20_scan_bringup`, executable `sensor_pose_adapter`.

Both receive the same parameter dictionary and use the same node name, and launch conditions guarantee that only one adapter starts.

## Error Handling

- TF lookup failures are throttled and retried until the configured wait timeout.
- Queue-full and TF-wait-timeout drops are throttled warnings.
- Invalid nonpositive retry periods and queue sizes fail parameter validation at startup rather than creating an unsafe runtime configuration.
- No transform interpolation policy or safety timeout is relaxed.

## Verification

Static and automated checks:

- Build the new C++ package and the existing bringup package.
- Add C++ unit tests for queue capacity, timeout removal, and successful matched publication where practical.
- Retain and run the existing Python tests.
- Verify launch syntax and both backend selections.

Runtime checks with command output disabled:

- Confirm `/rslidar_points_front` has exactly one adapter subscriber.
- Confirm `/scan/front_cloud_stamp`, `/scan/sensor_pose`, and `/scan/front_cloud_synced` retain their types and publishers.
- Compare driver rate with `/scan/front_cloud_stamp`; target a stable rate close to the driver's approximately 10 Hz.
- Check the maximum observed stamp gap against the existing 0.30-second stale threshold.
- Check adapter CPU and logs for TF pending or dropped clouds.
- Keep the UDP bridge `DISARMED` throughout verification.

## Rollback

Set `sensor_pose_adapter_backend:=python` to use the previous implementation without reverting code. A Git revert of the launch and new-package commit provides a complete rollback.
