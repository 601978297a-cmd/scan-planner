#!/usr/bin/env bash
set -eo pipefail
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner版本1/Super-LIO/install/setup.bash"

for setup_file in "$ROS_SETUP" "$SUPER_LIO_SETUP"; do
    if [[ ! -f "$setup_file" ]]; then
        echo "缺少构建环境: $setup_file" >&2
        echo "请先编译对应工作区。" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"

exec ros2 launch super_lio relocation_points.py
