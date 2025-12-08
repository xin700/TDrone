#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source OpenVINO setup (try 2024 first, then 2023)
if [ -f /opt/intel/openvino_2024/setupvars.sh ]; then
    source /opt/intel/openvino_2024/setupvars.sh
elif [ -f /opt/intel/openvino_2023/setupvars.sh ]; then
    source /opt/intel/openvino_2023/setupvars.sh
fi

# Source workspace setup if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
