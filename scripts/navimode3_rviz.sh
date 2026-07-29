#!/usr/bin/env bash
set -eo pipefail

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner_test/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner_test/SCAN-Planner/install/setup.bash"
MAP_YAML="/home/nvidia/Super-LIO/src/super_lio/map/map.yaml"
MAP_IMAGE="/home/nvidia/Super-LIO/src/super_lio/map/map.pgm"
RVIZ_CONFIG="/home/nvidia/scanplanner_test/SCAN-Planner/src/"\
"m20_scan_bringup/rviz/m20_scan.rviz"
navigation_pid=""
rviz_pid=""

for required_file in \
    "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP" \
    "$MAP_YAML" "$MAP_IMAGE" "$RVIZ_CONFIG"; do
    if [[ ! -f "$required_file" ]]; then
        echo "Missing file: $required_file" >&2
        exit 1
    fi
done

cleanup() {
    trap - EXIT INT TERM

    for pid in "$rviz_pid" "$navigation_pid"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid"
        fi
    done

    for pid in "$rviz_pid" "$navigation_pid"; do
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

setsid ros2 launch m20_scan_bringup \
    m20_mode3_navigation.launch.py \
    map:="$MAP_YAML" &
navigation_pid=$!

setsid rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!

wait -n "$navigation_pid" "$rviz_pid"
