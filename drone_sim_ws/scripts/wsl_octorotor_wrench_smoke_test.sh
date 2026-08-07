#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install
source install/setup.bash

gazebo_log="/tmp/my_drone_octorotor_wrench.log"
setsid ros2 launch drone_arm_sim octorotor_sim.launch.py spawn_z:=1.0 \
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
setsid ros2 run drone_arm_sim gazebo_wrench_controller \
  --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_octorotor_example.urdf" \
  --entity-name "my_drone::base_link" \
  >/tmp/my_drone_octorotor_wrench_controller.log 2>&1 &
controller_pid=$!
sleep 5

timeout 60 ros2 run drone_arm_sim hover_acceptance \
  --settle-time 20 \
  --position-tolerance 0.05 \
  --attitude-tolerance-deg 2.0

echo "my_drone octorotor wrench hover smoke test passed"
