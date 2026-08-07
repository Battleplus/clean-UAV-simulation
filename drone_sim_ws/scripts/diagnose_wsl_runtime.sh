#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "PATH=${PATH}"
echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
command -v ros2 || true
command -v colcon || true

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
source install/setup.bash
echo "AFTER_INSTALL_AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
ros2 pkg prefix drone_arm_sim || true
python3 -c 'import drone_arm_sim; print("PYTHON_PACKAGE=" + drone_arm_sim.__file__)' || true
