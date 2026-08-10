#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

robot_file="${ROBOT_FILE:-${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf}"
config_file="${CONFIG_FILE:-${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json}"
result_log="${RESULT_LOG:-${workspace_dir}/analysis/formal_rotor_identification.log}"
: >"${result_log}"

cleanup_runtime() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill -- "-${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
  fi
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  pkill -x ros2 2>/dev/null || true
}
trap cleanup_runtime EXIT
cleanup_runtime

for rotor in $(seq 1 8); do
  gazebo_log="/tmp/my_drone_formal_rotor_${rotor}_gazebo.log"
  MY_DRONE_URDF="${robot_file}" setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
    headless:=true enable_controller:=false enable_arm_control:=false spawn_z:=10 \
    config_file:="${config_file}" >"${gazebo_log}" 2>&1 &
  launch_pid=$!

  ready=false
  for _ in $(seq 1 40); do
    if timeout 2 ros2 topic echo /model/my_drone/odometry --once \
      >/dev/null 2>&1; then
      ready=true
      break
    fi
    sleep 1
  done
  if [[ "${ready}" != "true" ]]; then
    echo "Rotor ${rotor}: Gazebo odometry did not become ready" >&2
    tail -n 80 "${gazebo_log}" >&2 || true
    exit 1
  fi

  echo "FORMAL_ROTOR_IDENTIFICATION_START rotor=${rotor}" | tee -a "${result_log}"
  set +e
  timeout 30 ros2 run drone_arm_sim gazebo_rotor_identification \
    --urdf "${robot_file}" --rotor-index "${rotor}" \
    --pulse-speed 400 --pulse-duration 0.20 \
    2>&1 | tee -a "${result_log}"
  identification_status=${PIPESTATUS[0]}
  set -e
  if [[ "${identification_status}" -ne 0 ]]; then
    echo "Rotor ${rotor}: identification failed with status ${identification_status}" >&2
    tail -n 80 "${gazebo_log}" >&2 || true
    exit 1
  fi

  cleanup_runtime
  if grep -Eiq "failed to load|could not load|error.*plugin|nan|abort" "${gazebo_log}"; then
    echo "Rotor ${rotor}: Gazebo log contains an error" >&2
    exit 1
  fi
  sleep 1
done

completion_count="$(grep -c 'ROTOR_IDENTIFICATION_COMPLETE' "${result_log}")"
rotor_count="$(grep -c 'rotor [1-8]:' "${result_log}")"
if [[ "${completion_count}" -ne 8 || "${rotor_count}" -ne 8 ]]; then
  echo "Expected eight rotor measurements, got completions=${completion_count} rotors=${rotor_count}" >&2
  exit 1
fi
echo "FORMAL_ROTOR_IDENTIFICATION_PASS log=${result_log}"
