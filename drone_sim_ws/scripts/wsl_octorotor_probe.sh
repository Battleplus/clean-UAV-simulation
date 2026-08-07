#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

log_path="/tmp/my_drone_octorotor_probe.log"
setsid ros2 launch drone_arm_sim octorotor_sim.launch.py \
  >"${log_path}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  if [[ -n "${motor_pid:-}" ]]; then
    kill "${motor_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 1
ros2 run drone_arm_sim gazebo_motor_controller \
  > /tmp/my_drone_motor_hover.log 2>&1 &
motor_pid=$!
sleep 10

echo "=== Motor command ==="
timeout 5 ros2 topic echo /my_drone/command/motor_speed --once
echo "=== Octorotor odometry ==="
timeout 5 ros2 topic echo /model/my_drone/odometry --once

if grep -Eiq "failed to load|could not load|error.*plugin|nan|abort" "${log_path}"; then
  echo "Gazebo octorotor log contains a plugin, NaN, or abort error." >&2
  exit 1
fi
