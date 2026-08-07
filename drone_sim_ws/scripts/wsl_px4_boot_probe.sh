#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
gazebo_log="/tmp/my_drone_px4_gazebo.log"
px4_log="/tmp/my_drone_px4_boot.log"

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
    kill "${bootstrap_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 1
ros2 run drone_arm_sim gazebo_wrench_controller \
  --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_octorotor_example.urdf" \
  --target 0 0 1 \
  --entity-name "my_drone::base_link" \
  >/tmp/my_drone_px4_bootstrap.log 2>&1 &
bootstrap_pid=$!

sleep 6
if ! gz topic -l | grep -q \
  "/world/flight_world/model/my_drone/link/base_link/sensor/imu_sensor/imu"; then
  echo "The my_drone IMU topic was not created." >&2
  tail -n 80 "${gazebo_log}" >&2
  exit 1
fi

cd "${px4_dir}"
(
  sleep 12
  timeout 4 gz topic -e -n 2 -t \
    /world/flight_world/model/my_drone/link/base_link/sensor/imu_sensor/imu \
    >/tmp/my_drone_px4_imu_transport.log 2>&1 || true
) &
transport_probe_pid=$!
{
  sleep 20
  echo "commander status"
  echo "gz_bridge status"
  echo "ekf2 status"
  echo "listener vehicle_imu 1"
  echo "listener vehicle_attitude 1"
  echo "listener estimator_status 1"
  echo "listener estimator_status_flags 1"
  echo "listener sensor_gyro 1"
  echo "listener sensor_accel 1"
  echo "listener sensor_baro 1"
  echo "listener sensor_mag 1"
  echo "listener sensor_gps 1"
  echo "listener vehicle_local_position 1"
  echo "listener actuator_outputs 1"
  sleep 2
  echo "shutdown"
} | env \
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone \
  PX4_SYS_AUTOSTART=4015 \
  timeout 50 ./build/px4_sitl_default/bin/px4 \
  >"${px4_log}" 2>&1 || px4_exit=$?

if [[ "${px4_exit:-0}" -ne 0 && "${px4_exit:-0}" -ne 124 ]]; then
  echo "PX4 exited with code ${px4_exit}." >&2
  tail -n 120 "${px4_log}" >&2
  exit "${px4_exit}"
fi

echo "=== PX4 boot evidence ==="
grep -E \
  "PX4 will attach|Ready for takeoff|WARN|ERROR|sensor_gps|vehicle_local_position|actuator_outputs|INFO  \\[commander\\]" \
  "${px4_log}" | tail -n 100 || true
echo "=== Gazebo IMU transport evidence ==="
grep -E "sec:|nsec:|linear_acceleration|angular_velocity" \
  /tmp/my_drone_px4_imu_transport.log | tail -n 30 || true

if grep -Eiq "failed to start|failed to connect|timed out waiting|ERROR \\[init\\]" "${px4_log}"; then
  echo "PX4 did not attach cleanly to the Gazebo model." >&2
  exit 1
fi

if ! grep -q "PX4 will attach to existing model" "${px4_log}"; then
  echo "PX4 did not select the existing-model startup path." >&2
  exit 1
fi

echo "PX4 boot probe passed; logs: ${gazebo_log}, ${px4_log}"
