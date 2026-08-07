#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe_id="${AIRFRAME_ID:-4015}"
config_file="${CONFIG_FILE:-${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/config/my_drone_v2_cad.json}"
use_bootstrap="${USE_BOOTSTRAP:-1}"
spawn_z="${SPAWN_Z:-1.0}"
acceptance_z="${ACCEPTANCE_Z:-2.0}"
gazebo_log="/tmp/my_drone_v2_px4_gazebo.log"
px4_log="/tmp/my_drone_v2_px4.log"
bootstrap_log="/tmp/my_drone_v2_px4_bootstrap.log"
acceptance_log="/tmp/my_drone_v2_px4_acceptance.log"
offboard_log="/tmp/my_drone_v2_px4_offboard.log"
motor_topic_log="/tmp/my_drone_v2_px4_motor_topic.log"
for path in "${gazebo_log}" "${px4_log}" "${bootstrap_log}" \
  "${acceptance_log}" "${offboard_log}" "${motor_topic_log}"; do
  : >"${path}"
done

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash

setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true enable_controller:=false spawn_z:="${spawn_z}" config_file:="${config_file}" \
  >"${gazebo_log}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  if [[ -n "${bootstrap_pid:-}" ]]; then
    kill -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

for _ in $(seq 1 50); do
  if gz topic -l 2>/dev/null | grep -q \
    "/model/my_drone/link/base_link/sensor/imu_sensor/imu"; then
    break
  fi
  sleep 1
done
if ! gz topic -l | grep -q \
  "/model/my_drone/link/base_link/sensor/imu_sensor/imu"; then
  echo "The CAD my_drone IMU topic was not created." >&2
  tail -n 120 "${gazebo_log}" >&2
  exit 1
fi

# Start the one-shot persistent gravity wrench only after the model entity
# exists; publishing it earlier silently targets a nonexistent link.
if [[ "${use_bootstrap}" -eq 1 ]]; then
  setsid ros2 run drone_arm_sim gazebo_wrench_controller \
    --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v2/my_drone_cad_dynamic.urdf" \
    --target 0 0 "${spawn_z}" \
    --entity-name "my_drone::base_link" \
    --persistent-bootstrap \
    >"${bootstrap_log}" 2>&1 &
  bootstrap_pid=$!
fi

# The movable CAD arm needs a short gravity-settling window before PX4 starts
# its at-rest gyro and magnetic-heading initialization.
sleep "${PRE_PX4_SETTLE_SECONDS:-15}"

timeout 45 ros2 topic echo /my_drone/command/motor_speed \
  >"${motor_topic_log}" 2>&1 &

cd "${px4_dir}"
{
  estimator_ready=0
  for _ in $(seq 1 100); do
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
  echo "param set NAV_DLL_ACT 0"
  echo "param set MC_YAW_P 1.0"
  echo "param set MC_YAW_WEIGHT 0.3"
  echo "param set MC_YAWRATE_K 0.3"
  echo "commander arm"
  sleep 2
  "${px4_dir}/.venv/bin/python" \
    "${workspace_dir}/scripts/px4_offboard_hover.py" \
    --down -1.0 --duration 85 >"${offboard_log}" 2>&1 &
  offboard_pid=$!
  sleep 3
  if [[ -n "${bootstrap_pid:-}" ]]; then
    kill -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  fi
  sleep 7
  timeout 100 ros2 run drone_arm_sim hover_acceptance \
    --target 0 0 "${acceptance_z}" \
    --settle-time 30 \
    --position-tolerance 0.35 \
    --attitude-tolerance-deg 7.0 \
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
  PX4_SYS_AUTOSTART="${airframe_id}" \
  timeout 190 ./build/px4_sitl_default/bin/px4 \
  >"${px4_log}" 2>&1 || px4_exit=$?

echo "=== PX4 CAD flight evidence ==="
tr -d '\033' <"${px4_log}" | grep -E \
  "Ready for takeoff|Armed|Takeoff detected|navigation mode|Preflight Fail|WARN|ERROR" \
  | tail -n 120 || true
echo "=== PX4 CAD hover acceptance ==="
cat "${acceptance_log}" 2>/dev/null || true
echo "=== bridged motor command samples ==="
tail -n 120 "${motor_topic_log}" 2>/dev/null || true
echo "=== direct motor node evidence ==="
grep -E "first nonzero.*motor command" "${gazebo_log}" | tail -n 10 || true

if [[ "${px4_exit:-0}" -ne 0 && "${px4_exit:-0}" -ne 124 ]]; then
  echo "PX4 exited with code ${px4_exit}." >&2
  tail -n 160 "${px4_log}" >&2
  exit "${px4_exit}"
fi
if ! grep -q "Ready for takeoff" "${px4_log}"; then
  echo "PX4 estimator did not become ready." >&2
  tail -n 160 "${px4_log}" >&2
  exit 1
fi
if ! grep -q "Armed by" "${px4_log}" \
  && ! grep -q "INFO  \[commander\] Armed" "${px4_log}"; then
  echo "PX4 did not arm." >&2
  exit 1
fi
if ! grep -q "passed=True" "${acceptance_log}"; then
  echo "PX4 CAD hover acceptance failed." >&2
  tail -n 160 "${gazebo_log}" >&2
  exit 1
fi

echo "PX4 CAD my_drone takeoff and hover passed"
