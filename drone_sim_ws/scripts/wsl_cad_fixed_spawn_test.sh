#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_path="/tmp/my_drone_v2_cad_fixed_spawn.log"
: >"${log_path}"

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

setsid ros2 launch drone_arm_sim cad_fixed_model.launch.py \
  headless:=true spawn_z:=0.35 >"${log_path}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if gz model --list 2>/dev/null | grep -q "my_drone_v2"; then
    break
  fi
  sleep 1
done

if ! gz model --list | grep -q "my_drone_v2"; then
  echo "my_drone_v2 was not created" >&2
  tail -n 100 "${log_path}" >&2
  exit 1
fi

pose="$(gz model -m my_drone_v2 -p)"
echo "=== Gazebo model pose ==="
echo "${pose}"
if [[ -z "${pose}" ]]; then
  echo "Gazebo did not return the model pose" >&2
  exit 1
fi

topics="$(gz topic -l)"
imu_topic="/world/flight_world/model/my_drone_v2/link/base_link/sensor/imu_sensor/imu"
if ! grep -qx "${imu_topic}" <<<"${topics}"; then
  echo "Expected IMU topic is missing: ${imu_topic}" >&2
  exit 1
fi
if ! grep -qx "/model/my_drone_v2/odometry" <<<"${topics}"; then
  echo "Expected odometry topic is missing" >&2
  exit 1
fi

if grep -Eiq "mesh.*(fail|error)|Unable to find|Error Code" "${log_path}"; then
  echo "Gazebo reported a mesh/model error" >&2
  grep -Ei "mesh.*(fail|error)|Unable to find|Error Code" "${log_path}" >&2
  exit 1
fi

echo "CAD fixed-model URDF spawned successfully with IMU and odometry"
