#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_path="/tmp/my_drone_v2_cad_direct_hover.log"
: >"${log_path}"

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true enable_controller:=true spawn_z:=1.0 target_ned_z:=-1.0 \
  >"${log_path}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 45); do
  if gz model --list 2>/dev/null | grep -q "my_drone"; then
    break
  fi
  sleep 1
done

if ! gz model --list | grep -q "my_drone"; then
  echo "CAD dynamic model did not spawn" >&2
  tail -n 120 "${log_path}" >&2
  exit 1
fi

timeout 45 ros2 run drone_arm_sim hover_acceptance \
  --target 0 0 1 \
  --settle-time 18 \
  --position-tolerance 0.20 \
  --attitude-tolerance-deg 6.0

echo "=== CAD direct-thrust evidence ==="
grep -E "mass=|first allocation|ERROR|Error|failed" "${log_path}" \
  | tail -n 40 || true
echo "CAD-derived split model direct-thrust hover passed"
