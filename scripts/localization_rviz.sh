#!/usr/bin/env bash
set -eo pipefail

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner_test/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner_test/SCAN-Planner/install/setup.bash"
RVIZ_CONFIG="/home/nvidia/scanplanner_test/SCAN-Planner/src/"\
"m20_scan_bringup/rviz/m20_scan.rviz"
localization_pid=""
rviz_pid=""

for required_file in \
    "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP" "$RVIZ_CONFIG"; do
    if [[ ! -f "$required_file" ]]; then
        echo "Missing file: $required_file" >&2
        exit 1
    fi
done

cleanup() {
    trap - EXIT INT TERM

    for pid in "$rviz_pid" "$localization_pid"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
        fi
    done

    for pid in "$rviz_pid" "$localization_pid"; do
        if [[ -n "$pid" ]]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT
trap 'exit 130' INT TERM

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"
source "$SCAN_SETUP"

ros2 launch super_lio relocation_points.py &
localization_pid=$!

rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!

wait -n "$localization_pid" "$rviz_pid"
