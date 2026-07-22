#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"

for setup_file in "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP"; do
    if [[ ! -f "$setup_file" ]]; then
        echo "缺少构建环境: $setup_file" >&2
        echo "请先编译对应工作区。" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"
source "$SCAN_SETUP"

exec ros2 launch m20_scan_bringup \
    m20_localization_navigation.launch.py
