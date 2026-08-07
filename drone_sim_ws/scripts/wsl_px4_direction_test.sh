#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
gazebo_log="/tmp/my_drone_px4_direction_gazebo.log"
px4_log="/tmp/my_drone_px4_direction.log"
bootstrap_log="/tmp/my_drone_px4_direction_bootstrap.log"
mission_log="/tmp/my_drone_px4_direction_mission.log"
: >"${gazebo_log}"
: >"${px4_log}"
: >"${bootstrap_log}"
: >"${mission_log}"

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install
source install/setup.bash

setsid ros2 launch drone_arm_sim octorotor_sim.launch.py spawn_z:=1.0 \
  >"${gazebo_log}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  if [[ -n "${bootstrap_pid:-}" ]]; then
    kill -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 1
setsid ros2 run drone_arm_sim gazebo_wrench_controller \
  --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_octorotor_example.urdf" \
  --target 0 0 1 \
  --entity-name "my_drone::base_link" \
  >"${bootstrap_log}" 2>&1 &
bootstrap_pid=$!
sleep 6

cd "${px4_dir}"
{
  sleep 5
  echo "param set NAV_DLL_ACT 0"
  estimator_ready=0
  for _ in $(seq 1 80); do
    if grep -q "Ready for takeoff" "${px4_log}"; then
      estimator_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${estimator_ready}" -ne 1 ]]; then
    echo "shutdown"
    exit 0
  fi

  echo "param set MC_YAW_P 1.0"
  echo "param set MC_YAW_WEIGHT 0.3"
  echo "param set MC_YAWRATE_K 0.3"
  echo "commander arm"
  sleep 2
  "${px4_dir}/.venv/bin/python" \
    "${workspace_dir}/scripts/px4_offboard_direction_test.py" \
    >"${mission_log}" 2>&1 &
  mission_pid=$!
  sleep 3
  kill -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  wait "${mission_pid}" || true
  echo "commander status"
  sleep 2
  echo "shutdown"
} | env \
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone \
  PX4_SYS_AUTOSTART=4015 \
  timeout 240 ./build/px4_sitl_default/bin/px4 \
  >"${px4_log}" 2>&1 || px4_exit=$?

echo "=== Direction mission ==="
cat "${mission_log}"
echo "=== PX4 state ==="
tr -d '\033' <"${px4_log}" | grep -E \
  "Ready for takeoff|Armed|Takeoff detected|navigation mode|Failsafe" \
  | tail -n 40 || true

if [[ "${px4_exit:-0}" -ne 0 && "${px4_exit:-0}" -ne 124 ]]; then
  exit "${px4_exit}"
fi
if ! grep -q "DIRECTION_TEST_PASS" "${mission_log}"; then
  echo "PX4 directional flight test failed." >&2
  exit 1
fi
echo "PX4 forward/right/back/left directional flight test passed"
