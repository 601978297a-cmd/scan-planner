#!/usr/bin/env bash
set -eo pipefail
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner_test/Super-LIO/install/setup.bash"
SCAN_SETUP="/home/nvidia/scanplanner_test/SCAN-Planner/install/setup.bash"
MAP_YAML="/home/nvidia/scanplanner_test/SCAN-Planner/install/"\
"m20_scan_bringup/share/m20_scan_bringup/config/m20_mode3_map.yaml"
MAP_IMAGE="/home/nvidia/Super-LIO/src/super_lio/map/map.pgm"

for required_file in \
    "$ROS_SETUP" "$SUPER_LIO_SETUP" "$SCAN_SETUP" "$MAP_YAML" "$MAP_IMAGE"; do
    if [[ ! -f "$required_file" ]]; then
        echo "缺少文件: $required_file" >&2
        echo "请先编译对应工作区并确认二维地图路径。" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"
source "$SCAN_SETUP"

exec ros2 launch m20_scan_bringup \
    m20_mode3_navigation.launch.py \
    map:="$MAP_YAML"
