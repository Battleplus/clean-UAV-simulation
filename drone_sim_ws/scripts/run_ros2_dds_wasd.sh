#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
echo "ROS2 DDS WASD controller"
echo "T 起飞 | W/S 前后 | A/D 左右 | R/F 升降 | Q/E 转向 | L 降落"
echo "O 退出Offboard | X连按两次紧急停机 | Z安全退出"
exec ros2 run px4_ros2_control dds_wasd_control
