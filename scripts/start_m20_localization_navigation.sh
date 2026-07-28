#!/usr/bin/env bash
set -eo pipefail
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner_test/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner_test/SCAN-Planner/install/setup.bash"
RVIZ_CONFIG="/home/nvidia/scanplanner_test/SCAN-Planner/src/"\
"m20_scan_bringup/rviz/m20_scan.rviz"

for setup_file in \
    "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP" "$RVIZ_CONFIG"; do
    if [[ ! -f "$setup_file" ]]; then
        echo "缺少文件: $setup_file" >&2
        echo "请先编译对应工作区。" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"
source "$SCAN_SETUP"

stack_pid=""
rviz_pid=""

cleanup() {
    trap - EXIT
    for child_pid in "$stack_pid" "$rviz_pid"; do
        if [[ -n "$child_pid" ]] && kill -0 "$child_pid" 2>/dev/null; then
            kill -INT "$child_pid" 2>/dev/null || true
        fi
    done
    for child_pid in "$stack_pid" "$rviz_pid"; do
        if [[ -n "$child_pid" ]]; then
            wait "$child_pid" 2>/dev/null || true
        fi
    done
}

trap 'exit 130' INT TERM
trap cleanup EXIT

ros2 launch m20_scan_bringup \
    m20_localization_navigation.launch.py \
    rviz:=false &
stack_pid=$!

rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!

wait -n "$stack_pid" "$rviz_pid"
