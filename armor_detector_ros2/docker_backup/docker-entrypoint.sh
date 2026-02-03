#!/bin/bash
set -e

# Add root to video and kvm groups for GPU access
usermod -a -G video,kvm root 2>/dev/null || true

# Create symbolic links for compatibility with old path
# Both /home/user and /root are needed since different contexts use different users
mkdir -p /home/user/droneAim /root/droneAim
ln -sf /ros2_ws /home/user/droneAim/TDrone 2>/dev/null || true
ln -sf /ros2_ws /root/droneAim/TDrone 2>/dev/null || true

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
