#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
runtime_dir="/tmp/my_drone_ros2_dds"
mkdir -p "${runtime_dir}"
model_settle_s="${MODEL_SETTLE_S:-8}"
px4_ready_settle_s="${PX4_READY_SETTLE_S:-5}"

# Keep the PX4 SITL build's generated airframe in sync with the authoritative
# project copy.  PX4 executes the file under build/.../etc at runtime; merely
# editing drone_sim_ws/px4/airframes would otherwise leave an older parameter
# set active until a full PX4 rebuild.
project_airframe="${PROJECT_AIRFRAME_FILE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../px4/airframes" && pwd)/4026_gz_my_drone_octorotor_7p735}"
px4_build_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/4026_gz_my_drone_octorotor_7p735"
if [[ -f "${project_airframe}" && -d "$(dirname "${px4_build_airframe}")" ]]; then
  cp "${project_airframe}" "${px4_build_airframe}"
fi

# A second launch would create another direct-wrench node subscribing to the
# same actuator topic and apply thrust twice to one Gazebo entity.
if [[ "${CLEAN_STALE_RUNTIME:-1}" == "1" ]]; then
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x gazebo_sensor_d 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  pkill -x px4 2>/dev/null || true
  pkill -x MicroXRCEAgent 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  # ROS launch and the DDS keyboard controller both have process name ros2.
  pkill -x ros2 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  # A PTY test may leave the installed Python entry point behind if the
  # parent shell is interrupted; never allow two DDS Offboard publishers.
  pkill -f '/px4_ros2_control/dds_wasd_control' 2>/dev/null || true
  # The arm keyboard is a plain bash loop (not a ros2 process), so a prior
  # visible terminal can otherwise survive a clean backend restart and leave
  # two operator consoles on screen.
  pkill -f '/run_ros2_arm_keyboard.sh' 2>/dev/null || true
  sleep 1
fi

source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
ros2_controllers_overlay="${workspace_dir}/.deps/ros2_controllers_debs/overlay/opt/ros/jazzy"
if [[ -d "${ros2_controllers_overlay}" ]]; then
  export AMENT_PREFIX_PATH="${ros2_controllers_overlay}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${ros2_controllers_overlay}/lib:${LD_LIBRARY_PATH:-}"
fi
source "${workspace_dir}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

agent_log="${runtime_dir}/agent.log"
gazebo_log="${runtime_dir}/gazebo.log"
px4_log="${runtime_dir}/px4.log"
arm_init_log="${runtime_dir}/arm_init.log"
settle_log="${runtime_dir}/settle.log"
px4_stability_log="${runtime_dir}/px4_stability.log"
: >"${agent_log}"
: >"${gazebo_log}"
: >"${px4_log}"
: >"${arm_init_log}"
: >"${settle_log}"
: >"${px4_stability_log}"

setsid /home/asus/.local/bin/MicroXRCEAgent udp4 -p 8888 -v 4 \
  >"${agent_log}" 2>&1 &
echo $! >"${runtime_dir}/agent.pid"

default_robot_file="${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
default_config_file="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json"
export MY_DRONE_URDF="${ROBOT_FILE:-${default_robot_file}}"
setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:="${HEADLESS:-false}" enable_controller:=false \
  enable_arm_control:="${ENABLE_ARM_CONTROL:-false}" \
  spawn_z:="${SPAWN_Z:-0.817}" \
  reaction_moment_ratio_m:="${REACTION_MOMENT_RATIO_M:--1}" \
  wind_enu_x:="${WIND_ENU_X:-nan}" wind_enu_y:="${WIND_ENU_Y:-nan}" \
  wind_enu_z:="${WIND_ENU_Z:-nan}" \
  battery_dynamics_enabled:="${BATTERY_DYNAMICS_ENABLED:-false}" \
  enable_sensor_delay:="${ENABLE_SENSOR_DELAY:-true}" \
  imu_delay_ms:="${IMU_DELAY_MS:-0}" \
  mag_delay_ms:="${MAG_DELAY_MS:-10}" \
  baro_delay_ms:="${BARO_DELAY_MS:-20}" \
  navsat_delay_ms:="${NAVSAT_DELAY_MS:-50}" \
  battery_internal_resistance_ohm:="${BATTERY_INTERNAL_RESISTANCE_OHM:-nan}" \
  battery_capacity_ah:="${BATTERY_CAPACITY_AH:-nan}" \
  battery_full_voltage_v:="${BATTERY_FULL_VOLTAGE_V:-nan}" \
  battery_empty_voltage_v:="${BATTERY_EMPTY_VOLTAGE_V:-nan}" \
  battery_minimum_loaded_voltage_v:="${BATTERY_MINIMUM_LOADED_VOLTAGE_V:-nan}" \
  battery_thrust_voltage_exponent:="${BATTERY_THRUST_VOLTAGE_EXPONENT:-nan}" \
  arm_torque_feedforward_enabled:="${ARM_TORQUE_FEEDFORWARD_ENABLED:-false}" \
  arm_torque_feedforward_max_delta_n:="${ARM_TORQUE_FEEDFORWARD_MAX_DELTA_N:-2.0}" \
  arm_static_com_feedforward_gain:="${ARM_STATIC_COM_FEEDFORWARD_GAIN:-0.0}" \
  arm_static_com_feedforward_time_constant_s:="${ARM_STATIC_COM_FEEDFORWARD_TIME_CONSTANT_S:-5.0}" \
  arm_coupling_target_mass_kg:="${ARM_COUPLING_TARGET_MASS_KG:-7.735}" \
  arm_payload_mass_kg:="${ARM_PAYLOAD_MASS_KG:-0.0}" \
  config_file:="${CONFIG_FILE:-${default_config_file}}" \
  >"${gazebo_log}" 2>&1 &
echo $! >"${runtime_dir}/gazebo.pid"

for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/model/my_drone/link/base_link/sensor/imu_sensor/imu'; then
    break
  fi
  sleep 1
done
# The CAD assembly rests on its vehicle-mounted landing gear and can bounce
# while the sensors first come online.  Let the rigid body settle before PX4 chooses its
# local-position origin; otherwise the first takeoff target contains the fall
# distance and the safety gate correctly aborts it.
echo "Waiting ${model_settle_s}s for the spawned CAD model to settle" >>"${gazebo_log}"
sleep "${model_settle_s}"
if ! python3 "${workspace_dir}/scripts/wait_model_settled.py" \
  --timeout "${MODEL_SETTLE_TIMEOUT_S:-45}" \
  --hold "${MODEL_SETTLE_HOLD_S:-2}" \
  --linear-limit "${MODEL_SETTLE_LINEAR_LIMIT_M_S:-0.08}" \
  --angular-limit "${MODEL_SETTLE_ANGULAR_LIMIT_RAD_S:-0.08}" \
  >"${settle_log}" 2>&1; then
  echo "Gazebo CAD model did not settle; refusing to start PX4" >&2
  cat "${settle_log}" >&2 || true
  tail -n 80 "${gazebo_log}" >&2 || true
  if [[ -f "${runtime_dir}/gazebo.pid" ]]; then
    kill -- "-$(cat "${runtime_dir}/gazebo.pid")" 2>/dev/null || true
  fi
  exit 1
fi

cd "${px4_dir}"
setsid env PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone PX4_SYS_AUTOSTART="${AIRFRAME_ID:-4026}" \
  ./build/px4_sitl_default/bin/px4 -d >"${px4_log}" 2>&1 &
echo $! >"${runtime_dir}/px4.pid"

for _ in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -Eq '^/fmu/out/vehicle_status(_v[0-9]+)?$'; then
    if [[ "${ENABLE_ARM_CONTROL:-false}" == "true" ]]; then
      # Position interfaces do not hold a gravity-loaded arm until the first
      # trajectory is received.  Freeze the documented CAD retracted pose
      # before any flight controller is allowed to arm.
      ros2 run drone_arm_sim arm_preset_control \
        --preset retracted --duration 8 --wait --tolerance 0.08 \
        >"${arm_init_log}" 2>&1
    fi
    # DDS topic discovery is not evidence that the estimator is ready.  In
    # particular, the formal 7.735 kg setup has occasionally reported a false
    # 0.5-0.6 m/s vertical velocity for several seconds after Gazebo itself is
    # already stationary.  Never let an automated test arm from that state.
    sleep "${px4_ready_settle_s}"
    if ! python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
      --horizontal "${PX4_READY_HORIZONTAL_LIMIT_M_S:-0.10}" \
      --vertical "${PX4_READY_VERTICAL_LIMIT_M_S:-0.08}" \
      --hold "${PX4_READY_STABLE_HOLD_S:-5}" \
      --timeout "${PX4_READY_STABLE_TIMEOUT_S:-90}" \
      >"${px4_stability_log}" 2>&1; then
      echo "PX4 local-position estimator did not stabilize; refusing to arm" >&2
      cat "${px4_stability_log}" >&2 || true
      exit 1
    fi
    echo "ROS2_DDS_NOARM_READY"
    echo "ROS2_DDS_READY arm_control=${ENABLE_ARM_CONTROL:-false}"
    echo "logs=${runtime_dir}"
    exit 0
  fi
  sleep 1
done

echo "ROS2 DDS topics did not appear within 90 seconds" >&2
tail -n 80 "${agent_log}" >&2 || true
tail -n 80 "${px4_log}" >&2 || true
exit 1
