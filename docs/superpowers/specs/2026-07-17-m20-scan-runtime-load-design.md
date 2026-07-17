# M20 SCAN Runtime Load Design

## Scope

Reduce SCAN navigation data starvation while keeping Super-LIO, the lidar
driver, SCAN core algorithms, UDP protocol, safety timeouts, and hardware
arming behavior unchanged.

## Observed Failure

The lidar driver reports approximately 10 Hz source and merged-cloud
publication. With the full SCAN RViz configuration enabled,
`/scan/front_cloud_stamp` falls to approximately 0.5 Hz. With RViz stopped it
recovers to approximately 5 Hz but still has occasional one-second gaps.

The host has eight CPU cores and is not continuously at 100 percent, but the
load average is close to eight. RViz, the Python sensor-pose adapter, SCAN, the
UDP bridge, Super-LIO, and the lidar driver contend for callback and DDS
processing time. The adapter also handles 200 Hz robot odometry and a full
PointCloud2 subscription in one single-threaded executor.

## Considered Approaches

1. Increase safety timeouts. Rejected because it would allow motion for up to
   one second without current obstacle data.
2. Replace the adapter with a new C++ package. This offers the best long-term
   efficiency but expands the change and build surface.
3. Optimize the existing Python adapter and default RViz load. Selected as the
   smallest reversible change that addresses the measured contention.

## Changes

### RViz

- Disable the raw front-lidar PointCloud2 display by default.
- Keep occupancy, planning markers, odometry, TF, and goal tools enabled.
- Reduce the RViz frame rate from 30 Hz to 10 Hz.
- Use `world` for the grid reference frame.
- Rename disabled legacy Lightning displays to their Super-LIO topics.

Operators can temporarily enable the raw front cloud for inspection, but it is
not part of the navigation view.

### Sensor Pose Adapter

- Use independent callback groups for odometry and cloud callbacks.
- Run the node in a two-thread `MultiThreadedExecutor`.
- Use depth-one best-effort QoS for high-rate odometry and cloud inputs so old
  samples do not queue behind current data.
- Protect the body-pose history with a lock and match clouds against a stable
  snapshot.
- Retain the full 200 Hz pose cache for timestamp matching.
- Rate-limit `/scan/body_pose_stamp` publication to 20 Hz. This remains well
  inside the existing 0.5-second safety timeout and reduces Python/DDS work in
  the UDP bridge.

## Safety

- `front_cloud_timeout`, `sensor_pose_timeout`, and timestamp matching
  tolerances are not relaxed.
- UDP remains manually armed and defaults to disabled.
- Tests and runtime validation do not arm the robot or send motion axes.

## Verification

1. Run package tests and lint on changed files.
2. Start SCAN, the real-capable UDP bridge in `DISARMED`, and the low-load RViz
   configuration.
3. Observe safety diagnostics for at least 30 seconds.
4. Require stable input rates and no repeated stale transition before any
   physical arming test.

