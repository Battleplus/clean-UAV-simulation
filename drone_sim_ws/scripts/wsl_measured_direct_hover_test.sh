#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_path="/tmp/my_drone_measured_direct_hover.log"
: >"${log_path}"

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

setsid ros2 launch drone_arm_sim measured_direct_thrust.launch.py \
  headless:=true enable_controller:=true spawn_z:=1.0 target_ned_z:=-1.0 \
  >"${log_path}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 10
timeout 35 ros2 run drone_arm_sim hover_acceptance \
  --target 0 0 1 \
  --settle-time 12 \
  --position-tolerance 0.15 \
  --attitude-tolerance-deg 5.0

echo "=== measured direct-thrust controller evidence ==="
grep -E "mass=|first allocation|ERROR|Error|failed" "${log_path}" \
  | tail -n 30 || true
echo "Measured-geometry direct-thrust Gazebo hover passed"
