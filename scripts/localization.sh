#!/usr/bin/env bash
set -eo pipefail

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
ROS_SETUP="/opt/ros/humble/setup.bash"
SUPER_LIO_SETUP="/home/nvidia/scanplanner_test/Super-LIO/install/setup.bash"

for required_file in "$ROS_SETUP" "$SUPER_LIO_SETUP"; do
    if [[ ! -f "$required_file" ]]; then
        echo "Missing file: $required_file" >&2
        exit 1
    fi
done

source "$ROS_SETUP"
source "$SUPER_LIO_SETUP"

exec ros2 launch super_lio relocation_points.py
