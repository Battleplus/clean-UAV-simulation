#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u

cd "${workspace_dir}"
colcon build --symlink-install
set +u
source install/setup.bash
set -u

python3 -m unittest discover \
  -s src/drone_arm_sim/test \
  -v

log_path="/tmp/my_drone_wrench_hover.log"
setsid ros2 launch drone_arm_sim wrench_hover.launch.py \
  >"${log_path}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 6
timeout 25 ros2 run drone_arm_sim hover_acceptance \
  --settle-time 8 \
  --position-tolerance 0.05 \
  --attitude-tolerance-deg 2.0

if grep -Eiq "failed to load|could not load|error.*plugin|nan" "${log_path}"; then
  echo "Gazebo log contains plugin or NaN errors: ${log_path}" >&2
  exit 1
fi

echo "my_drone wrench-hover runtime smoke test passed"
