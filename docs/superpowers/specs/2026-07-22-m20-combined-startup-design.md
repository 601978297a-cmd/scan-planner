# M20 Localization and SCAN Combined Startup Design

## Goal

Provide simple commands in `/home/nvidia/scanplanner版本1` so the operator can
start localization and navigation together, while starting RViz separately.
The commands must use the stable Super-LIO worktree and the current SCAN Planner
worktree rather than an older build from another directory.

## Operator Commands

Create two convenience links in `/home/nvidia/scanplanner版本1`:

- `./启动定位和导航.sh` starts Super-LIO relocation and SCAN Planner.
- `./启动RViz.sh` starts only the SCAN Planner RViz configuration.

RViz is intentionally excluded from the combined startup command. Pressing
Ctrl+C in the combined command must stop both localization and navigation.

## Versioned Files

Keep the actual scripts and ROS launch file in the SCAN Planner repository so
their behavior is version-controlled. The Chinese-named files in the parent
directory are symbolic links to those scripts.

Add:

- `scripts/start_m20_localization_navigation.sh`
- `scripts/start_m20_rviz.sh`
- `src/m20_scan_bringup/launch/m20_localization_navigation.launch.py`

## Combined ROS Launch

The combined launch includes two existing launch descriptions:

1. `super_lio/relocation.py` with `rviz:=false`.
2. `m20_scan_bringup/m20_scan_dry_run.launch.py` with
   `control_backend:=udp` and `enable_udp_output:=true`.

Using one ROS launch service gives both subsystems one process group and normal
ROS shutdown handling. No localization, TF, map, planner, auto-ARM, or UDP
control parameters are changed by this work.

## Workspace Selection

The scripts source these exact workspaces in order:

1. `/opt/ros/humble/setup.bash`
2. `/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash`
3. `/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash`

Each script checks that the required setup files exist and prints a direct
error instead of silently falling back to `/home/nvidia/Super-LIO`.

The stable Super-LIO worktree currently has source code but no install space.
Build it once during implementation before exposing the startup command.

## RViz

The RViz script sources the same two workspaces and opens
`src/m20_scan_bringup/rviz/m20_scan.rviz`. It does not start or stop localization,
navigation, or UDP output.

## Verification

Verification is non-actuating:

- run `bash -n` on both scripts;
- verify both symbolic links resolve to the versioned scripts;
- verify the combined launch includes relocation with RViz disabled and SCAN
  with the UDP backend enabled;
- build the stable Super-LIO workspace;
- build `m20_scan_bringup` and run its complete test suite;
- do not launch real UDP motion as part of automated verification.
