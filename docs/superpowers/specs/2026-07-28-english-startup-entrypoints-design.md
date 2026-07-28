# English Startup Entrypoints Design

## Goal

Replace the top-level Chinese startup links with two English commands:

- `./localization_rviz.sh`
- `./navimode3.sh`

The first command starts Super-LIO localization and RViz together. The second
starts the existing Mode 3 navigation stack.

## Process behavior

`localization_rviz.sh` directly configures the environment, starts Super-LIO
localization and RViz as child processes, and waits until either one exits. On
`Ctrl+C`, or when either child exits, it terminates and waits for both children
so no localization or RViz process is left behind. It does not invoke another
shell script.

`navimode3.sh` directly configures the environment and starts the Mode 3 ROS 2
launch file. It does not invoke another shell script.

## Files and compatibility

Add two tracked direct startup scripts:

`scripts/localization_rviz.sh`

`scripts/navimode3.sh`

At `/home/nvidia/scanplanner_test`, create the two English links and remove all
top-level `启动*.sh` links. Keep the existing English scripts under
`SCAN-Planner/scripts` because they remain the implementation targets.

Mode 1 source code and the localization, navigation, and RViz launch files are
not changed.

## Verification

- Validate all shell scripts with `bash -n`.
- Add tests for both English entrypoints and cleanup behavior in the supervisor.
- Confirm no top-level Chinese startup links remain.
- Run the `m20_scan_bringup` test suite on the robot.
