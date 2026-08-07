#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

result_log="/tmp/my_drone_rotor_identification_result.log"
: >"${result_log}"

cleanup() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

for rotor in $(seq 1 8); do
  gazebo_log="/tmp/my_drone_rotor_${rotor}_gazebo.log"
  setsid ros2 launch drone_arm_sim octorotor_sim.launch.py spawn_z:=10 \
    >"${gazebo_log}" 2>&1 &
  launch_pid=$!
  sleep 1
  timeout 15 ros2 run drone_arm_sim gazebo_rotor_identification \
    --rotor-index "${rotor}" \
    --pulse-duration 0.20 \
    2>&1 | tee -a "${result_log}"
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" 2>/dev/null || true
  launch_pid=""
  if grep -Eiq "failed to load|could not load|error.*plugin|nan|abort" "${gazebo_log}"; then
    echo "Gazebo rotor ${rotor} log contains an error." >&2
    exit 1
  fi
  sleep 1
done

completion_count="$(grep -c "ROTOR_IDENTIFICATION_COMPLETE" "${result_log}")"
if [[ "${completion_count}" -ne 8 ]]; then
  echo "Expected 8 completed rotor identifications, got ${completion_count}." >&2
  exit 1
fi
