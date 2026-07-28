#!/usr/bin/env bash
set -eo pipefail
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner版本1/SCAN-Planner/install/setup.bash"
RVIZ_CONFIG="/home/nvidia/scanplanner版本1/SCAN-Planner/src/"\
"m20_scan_bringup/rviz/m20_scan.rviz"

for required_file in \
    "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP" "$RVIZ_CONFIG"; do
    if [[ ! -f "$required_file" ]]; then
        echo "缺少文件: $required_file" >&2
        echo "请先编译对应工作区。" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"
source "$SCAN_SETUP"

exec rviz2 -d "$RVIZ_CONFIG"
