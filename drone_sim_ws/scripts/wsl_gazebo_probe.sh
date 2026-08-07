#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

log_path="/tmp/my_drone_gazebo_probe.log"
setsid ros2 launch drone_arm_sim wrench_hover.launch.py \
  enable_controller:=true \
  physics_engine:=gz-physics-dartsim-plugin \
  >"${log_path}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 8
echo "=== Gazebo topics ==="
gz topic -l | grep -E "my_drone|clock|stats" || true
echo "=== ROS topics ==="
ros2 topic list
echo "=== One Gazebo odometry sample ==="
timeout 5 gz topic -e -t /model/my_drone/odometry -n 1 || true
echo "=== One ROS odometry sample ==="
timeout 5 ros2 topic echo /model/my_drone/odometry --once || true
