# M20 Lightning-LM to SCAN-Planner Dry-Run Report

> Historical report: this records the original 2026-07-15 Lightning dry-run.
> The current M20 bringup uses Super-LIO `/lio/robo/odom`, global frame
> `world`, and front obstacle cloud `/rslidar_points_front`. The SCAN
> controller still publishes only `/scan/cmd_vel_debug`; guarded hardware
> outputs remain disabled by default. See
> `docs/superpowers/specs/2026-07-17-m20-scan-super-lio-backend-design.md`.

Date: 2026-07-15
Host: 192.168.0.174 / nvidia-desktop / ROS 2 Humble
Workspace: `/home/nvidia/scan_interface_audit/SCAN-Planner`
Branch: `ros2-community`
Commit: `d0b921c`

No `/NAV_CMD`, `/GAIT`, or real `/cmd_vel` messages were published.

## Created Files

New package: `src/m20_scan_bringup`

- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/m20_scan_bringup`
- `m20_scan_bringup/__init__.py`
- `m20_scan_bringup/sensor_pose_adapter.py`
- `launch/m20_scan_dry_run.launch.py`
- `config/m20_scan_planner.yaml`
- `config/m20_scan_controller.yaml`
- `rviz/m20_scan.rviz`
- `logs/dry_run_launch.log`
- `logs/rviz.log`
- `logs/final_checks.txt`

`git status --short`:

```text
?? src/m20_scan_bringup/
```

No SCAN core search, B-spline, FSM, or controller algorithm files were modified.

## Build

Installed missing system dependency:

```bash
sudo apt-get install -y libglm-dev
```

Build command:

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/scan_interface_audit/SCAN-Planner
colcon build --symlink-install --packages-up-to scan_planner m20_scan_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

```text
13 packages finished
```

Warnings observed were existing compile warnings plus OpenCV link warnings from `cv_bridge`; no build failure remained.

## Launch Commands

Dry-run:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/scan_interface_audit/SCAN-Planner/install/setup.bash
ros2 launch m20_scan_bringup m20_scan_dry_run.launch.py
```

Actual running process was started with `nohup`; PID is recorded in:

```text
src/m20_scan_bringup/logs/dry_run_launch.pid
```

RViz2:

```bash
DISPLAY=:0 rviz2 -d /home/nvidia/scan_interface_audit/SCAN-Planner/install/m20_scan_bringup/share/m20_scan_bringup/rviz/m20_scan.rviz
```

RViz2 is running on DISPLAY `:0`; PID is recorded in:

```text
src/m20_scan_bringup/logs/rviz.pid
```

## Running Nodes

Expected dry-run nodes are running:

```text
/sensor_pose_adapter
/scan_planner_node
/closed_loop_controller
```

Forbidden nodes were not present:

```text
/open_loop_controller
/go2_kinematic_sim
/map_generator
/mockamap
```

## Actual SCAN Wiring

`/scan_planner_node` subscribes to:

```text
/lightning/odom
/lightning/current_scan
/scan/sensor_pose
/move_base_simple/goal
/planning/go2_execution_frozen
```

`/closed_loop_controller` subscribes to:

```text
/lightning/odom
/planning/bspline
```

`/closed_loop_controller` publishes only:

```text
/scan/cmd_vel_debug
/planning/go2_execution_frozen
```

No real `/cmd_vel` remap is used in this launch.

## Parameters Verified

```text
grid_map.frame_id: map
grid_map.cloud_is_world: true
grid_map.need_extrinsic: false
```

Controller limits:

```text
max_vx: 0.15
max_vy: 0.08
max_vyaw: 0.20
```

Planner limits:

```text
manager.max_vel: 0.20
manager.max_acc: 0.20
optimization.max_vel: 0.20
optimization.max_acc: 0.20
```

## Topic Frequencies

Observed during final checks:

| Topic | Result |
|---|---|
| `/lightning/odom` | about 6-7 Hz in final check; earlier about 9 Hz |
| `/lightning/current_scan` | about 6-8 Hz |
| `/grid_map/occupancy` | about 18-20 Hz |
| `/grid_map/occupancy_inflate` | about 12-20 Hz |
| `/scan/cmd_vel_debug` | about 100 Hz |
| `/scan/sensor_pose` | not stable; see blocker below |

## Sensor Pose Adapter

Adapter behavior implemented:

- Subscribes to `/lightning/current_scan`.
- For each cloud stamp, queries TF `map -> lidar_link`.
- Publishes `/scan/sensor_pose` as `nav_msgs/msg/Odometry`.
- Uses `header.frame_id=map`, `child_frame_id=lidar_link`.
- Does not publish zero pose or latest-pose fallback when TF lookup fails.
- Uses a threaded TF listener so TF can update while cloud callbacks wait.

A valid sample was captured:

```yaml
header:
  frame_id: map
child_frame_id: lidar_link
pose.pose.position:
  x: 0.050662146895415004
  y: -0.0025508258226549577
  z: -0.03509225942294906
```

Blocking issue: Lightning/TF is not continuously providing a current `map -> lidar_link` transform at the cloud timestamp. Later logs repeatedly show:

```text
Lookup would require extrapolation into the future.
Requested time ... but the latest data is at time 1784103562.599140
```

Because the adapter intentionally refuses to publish fallback poses, `/scan/sensor_pose` is not stable. This must be fixed before true robot operation. Options:

1. Make Lightning publish continuous TF `map -> base_link` synchronized with `/lightning/odom`.
2. Add a dedicated dry-run-safe odom-to-TF bridge from `/lightning/odom` to `map -> base_link`, while avoiding duplicate TF publishers.
3. Change the adapter design to derive lidar pose from `/lightning/odom` plus static `base_link -> lidar_link`, but that is a different contract than pure TF lookup.

## Planning Test

A dry-run near goal was sent to `/move_base_simple/goal`:

```yaml
frame_id: map
position:
  x: 1.0
  y: 0.0
  z: 0.0
orientation:
  w: 1.0
```

This is not a robot motion command and was not sent to `/NAV_CMD`, `/GAIT`, or `/cmd_vel`.

Result:

- `/planning/bspline` published.
- `closed_loop_controller` received trajectories.
- `/scan/cmd_vel_debug` published at about 100 Hz.
- Logs showed `final_plan_success=1`.

Example B-spline control points included current pose near `x=0.05` and goal near `x=1.0`:

```text
pos_pts x range: about -0.11 -> 1.04
pos_pts z range: about -0.037 -> -0.033
```

Example debug command:

```yaml
linear:
  x: 0.14999346274203176
  y: 0.0014004051752011292
  z: 0.0
angular:
  z: 0.0
```

## RViz

RViz2 is running on `DISPLAY=:0` with:

```text
m20_scan_bringup/rviz/m20_scan.rviz
```

RViz log after QoS fix:

```text
OpenGl version: 4.6
```

No PointCloud2 QoS warning remained after changing point cloud displays to Best Effort.

Configured displays include:

- TF
- `/lightning/current_scan`
- `/grid_map/occupancy`
- `/grid_map/occupancy_inflate`
- `/grid_map/sliding_map_bbox`
- `/goal_point`
- `/init_list`
- `/optimal_list`
- `/a_star_list`
- `/lightning/path`

Note: `/planning/bspline` is a custom message and cannot be displayed directly by stock RViz; SCAN Marker topics are used for trajectory visualization.

## Safety Check

Final `/NAV_CMD` state:

```text
Publisher count: 0
Subscription count: 1
```

No `/GAIT` or real `/cmd_vel` publication was performed by this dry-run launch.

## Static Obstacle Test

Not performed in this run because it requires physically placing a box in front of the robot and selecting a side-behind target in RViz. The dry-run pipeline is ready for that check, but the result is pending manual setup.

## Warnings and Errors

Important warnings/errors:

- `sensor_pose_adapter`: repeated future extrapolation for `map -> lidar_link` after Lightning TF stopped updating at a stale timestamp.
- `scan_planner_node`: during replanning, some A-star attempts failed but later replans succeeded with `final_plan_success=1`.
- `ros2 topic hz` occasionally printed `sequence size exceeds remaining buffer`; this is DDS/tool noise seen on this machine.
- There are duplicate node-name warnings in the broader ROS graph unrelated to this dry-run package.

## True-Robot Blockers Before Connecting /NAV_CMD

1. Fix continuous synchronized TF for `map -> base_link -> lidar_link`, or change the adapter contract explicitly.
2. Add odom/cloud/trajectory timeout protection before any command bridge.
3. Do not trust 5-10 Hz odometry directly for a 100 Hz controller without a safety gate.
4. Implement `Twist -> drdds/msg/NavCmd` bridge only after dry-run stability is confirmed.
5. Keep `/scan/cmd_vel_debug` separate from real `/cmd_vel` and `/NAV_CMD` until a physical E-stop and gating policy are verified.
6. Recalculate SCAN body/inflation parameters for the actual M20 footprint; current values are Go2-derived dry-run defaults.

## Verdict

Dry-run SCAN integration is partially successful:

- Lightning odom is connected to SCAN.
- Lightning map-frame point cloud is connected to SCAN.
- SCAN occupancy maps are publishing.
- A near goal generated `/planning/bspline`.
- closed-loop debug velocity is publishing only on `/scan/cmd_vel_debug`.
- `/NAV_CMD` publisher count remained 0.
- RViz2 is running with a dedicated config.

Remaining issue: `/scan/sensor_pose` is not stable because Lightning TF is stale relative to point cloud timestamps. This is a blocker before any real robot command bridge.
