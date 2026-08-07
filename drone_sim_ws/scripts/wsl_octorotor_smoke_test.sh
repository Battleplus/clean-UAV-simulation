#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install
source install/setup.bash

gazebo_log="/tmp/my_drone_octorotor_hover.log"
controller_log="/tmp/my_drone_octorotor_controller.log"
setsid ros2 launch drone_arm_sim octorotor_sim.launch.py \
  >"${gazebo_log}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  if [[ -n "${controller_pid:-}" ]]; then
    kill -- "-${controller_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 1
setsid ros2 run drone_arm_sim gazebo_motor_controller \
  >"${controller_log}" 2>&1 &
controller_pid=$!
sleep 5

timeout 30 ros2 run drone_arm_sim hover_acceptance \
  --settle-time 6 \
  --position-tolerance 0.05 \
  --attitude-tolerance-deg 2.0

if grep -Eiq "failed to load|could not load|error.*plugin|nan|abort" "${gazebo_log}"; then
  echo "Gazebo octorotor log contains a plugin, NaN, or abort error." >&2
  exit 1
fi

echo "my_drone octorotor closed-loop hover smoke test passed"
