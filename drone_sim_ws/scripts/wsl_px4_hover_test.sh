#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
gazebo_log="/tmp/my_drone_px4_hover_gazebo.log"
px4_log="/tmp/my_drone_px4_hover.log"
bootstrap_log="/tmp/my_drone_px4_hover_bootstrap.log"
acceptance_log="/tmp/my_drone_px4_hover_acceptance.log"
offboard_log="/tmp/my_drone_px4_offboard.log"
: >"${gazebo_log}"
: >"${px4_log}"
: >"${bootstrap_log}"
: >"${acceptance_log}"
: >"${offboard_log}"

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
if ! gz topic -l | grep -q \
  "/world/flight_world/model/my_drone/link/base_link/sensor/imu_sensor/imu"; then
  echo "The my_drone IMU topic was not created." >&2
  exit 1
fi

cd "${px4_dir}"
{
  # The external wrench only holds the free-flying model still while PX4's
  # estimator initializes. PX4 is the sole controller after takeoff starts.
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
    echo "ekf2 status"
    echo "listener vehicle_local_position 1"
    echo "listener estimator_status_flags 1"
    echo "listener sensor_gyro 1"
    echo "listener sensor_baro 1"
    echo "listener sensor_mag 1"
    echo "listener sensor_gps 1"
    sleep 3
    echo "shutdown"
    exit 0
  fi
  echo "param set MC_YAW_P 1.0"
  echo "param set MC_YAW_WEIGHT 0.3"
  echo "param set MC_YAWRATE_K 0.3"
  echo "commander arm"
  sleep 2
  "${px4_dir}/.venv/bin/python" \
    "${workspace_dir}/scripts/px4_offboard_hover.py" \
    --down -1.0 --duration 80 \
    >"${offboard_log}" 2>&1 &
  offboard_pid=$!
  sleep 3
  kill -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  sleep 5
  timeout 100 ros2 run drone_arm_sim hover_acceptance \
    --target 0 0 2 \
    --settle-time 25 \
    --position-tolerance 0.25 \
    --attitude-tolerance-deg 5.0 \
    >"${acceptance_log}" 2>&1 || acceptance_exit=$?
  kill "${offboard_pid}" >/dev/null 2>&1 || true
  echo "commander status"
  echo "listener vehicle_local_position 1"
  echo "listener actuator_outputs 1"
  sleep 2
  echo "shutdown"
} | env \
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone \
  PX4_SYS_AUTOSTART=4015 \
  timeout 160 ./build/px4_sitl_default/bin/px4 \
  >"${px4_log}" 2>&1 || px4_exit=$?

if [[ "${px4_exit:-0}" -ne 0 && "${px4_exit:-0}" -ne 124 ]]; then
  echo "PX4 exited with code ${px4_exit}." >&2
  tail -n 120 "${px4_log}" >&2
  exit "${px4_exit}"
fi

echo "=== PX4 flight evidence ==="
tr -d '\033' <"${px4_log}" | grep -E \
  "Ready for takeoff|Armed|Takeoff detected|navigation mode|Preflight Fail|WARN|ERROR" \
  | tail -n 100 || true
echo "=== Hover acceptance ==="
cat "${acceptance_log}" 2>/dev/null || true

if ! grep -q "Ready for takeoff" "${px4_log}"; then
  echo "PX4 estimator did not become ready." >&2
  exit 1
fi
if ! grep -q "Armed by" "${px4_log}" && ! grep -q "INFO  \\[commander\\] Armed" "${px4_log}"; then
  echo "PX4 did not arm." >&2
  exit 1
fi
if ! grep -q "passed=True" "${acceptance_log}"; then
  echo "PX4 hover acceptance failed." >&2
  exit 1
fi

echo "PX4 my_drone 1 m relative takeoff and hover test passed"
