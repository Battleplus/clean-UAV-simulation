#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH:-}"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
echo "ROS2 DDS WASD controller"
echo "T takeoff | W/S forward/back | A/D left/right | R/F up/down"
echo "Q/E yaw | H brake/hover (preserve commanded altitude) | L land | O exit Offboard"
echo "Press X twice for emergency disarm | Z safe exit"
exec ros2 run px4_ros2_control dds_wasd_control
