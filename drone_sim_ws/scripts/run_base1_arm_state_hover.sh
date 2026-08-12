#!/usr/bin/env bash
# Paired Base 1 hover: arm control disabled vs enabled and held retracted.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis/base1"
runtime_dir="/tmp/my_drone_ros2_dds"
run_id="${1:-$(date +%Y%m%d_%H%M%S)}"

if [[ "${RUN_BASE1_ARM_STATE_CONFIRM:-0}" != "1" ]]; then
  echo "REFUSED: this paired test replaces the active Gazebo/PX4 runtime." >&2
  echo "Re-run with RUN_BASE1_ARM_STATE_CONFIRM=1." >&2
  exit 64
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
mkdir -p "${analysis_dir}"

px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
px4_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/4027_gz_my_drone_octorotor_debug_4kg"
cp "${airframe}" "${px4_airframe}"
chmod +x "${px4_airframe}"

logs=()
overall=0
for mode in disabled static; do
  label="base1_arm_state_${run_id}_${mode}"
  backend_log="${analysis_dir}/${label}_backend.log"
  flight_log="${analysis_dir}/${label}_flight.log"
  env_log="${analysis_dir}/${label}_environment.txt"
  logs+=("${flight_log}")

  export HEADLESS=true CLEAN_STALE_RUNTIME=1 PX4_FRESH_WORKDIR=1
  export ENABLE_ARM_CONTROL=false
  [[ "${mode}" == "static" ]] && export ENABLE_ARM_CONTROL=true
  export GZ_RANDOM_SEED=4027 MODEL_SETTLE_S=8 MODEL_SETTLE_HOLD_S=2
  # A 5 s gate occasionally passed before the barometric EKF completed a
  # delayed reset, after which a stationary vehicle reported several m/s of
  # vertical motion.  Wait longer before sampling and require ten continuous
  # seconds at the original strict velocity limits.  Thresholds are unchanged.
  export PX4_READY_SETTLE_S=20 PX4_READY_STABLE_HOLD_S=10
  # Keep the same strict velocity gate, but allow the PX4 estimator longer to
  # converge after the arm controller initializes.  This changes no flight
  # threshold and the aircraft remains disarmed throughout the wait.
  export PX4_READY_STABLE_TIMEOUT_S="${PX4_READY_STABLE_TIMEOUT_S:-180}"
  export ARM_FLIGHT_PROFILE=base1_static_4kg
  export ARM_FEEDFORWARD_ENABLED=false ARM_TORQUE_FEEDFORWARD_ENABLED=false
  export ARM_REACTION_TORQUE_FEEDFORWARD_GAIN=0.0
  export ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0
  export ARM_DISTURBANCE_OBSERVER_ENABLED=false
  export PX4_TOUCHDOWN_DISARM_ENABLED=true
  export PX4_TOUCHDOWN_DISARM_HEIGHT_M=0.20 PX4_TOUCHDOWN_DISARM_HOLD_S=0.5
  export AIRFRAME_ID=4027 PROJECT_AIRFRAME_FILE="${airframe}"
  export ROBOT_FILE="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
  export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
  export MY_DRONE_WORLD="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
  export ARM_COUPLING_TARGET_MASS_KG=4.0 BATTERY_DYNAMICS_ENABLED=false
  export ENABLE_SENSOR_DELAY=true IMU_DELAY_MS=0 MAG_DELAY_MS=0 BARO_DELAY_MS=0 NAVSAT_DELAY_MS=0
  export REACTION_MOMENT_RATIO_M=0.005 PX4_WASD_VERTICAL_SPEED_M_S=0.15 SPAWN_Z=0.289

  {
    echo "base1_ref=base-1"
    echo "base1_commit=b340ed6"
    echo "scope=4kg_only"
    echo "mode=${mode}"
    echo "ENABLE_ARM_CONTROL=${ENABLE_ARM_CONTROL}"
    echo "compensation=all_off"
    echo "seed=${GZ_RANDOM_SEED}"
  } >"${env_log}"

  echo "BASE1_ARM_STATE_BEGIN ${mode}"
  if ! bash "${workspace_dir}/scripts/wsl_start_ros2_dds_noarm.sh" 2>&1 | tee "${backend_log}"; then
    overall=1
    : >"${flight_log}"
    for name in gazebo px4 agent settle px4_stability arm_init; do
      [[ -f "${runtime_dir}/${name}.log" ]] && \
        cp "${runtime_dir}/${name}.log" "${analysis_dir}/${label}_${name}.log"
    done
    continue
  fi
  set +e
  python3 "${workspace_dir}/scripts/test_ros2_dds_dynamic_hover_pty.py" \
    --timeout "${BASE1_HOVER_TIMEOUT_S:-180}" \
    --settled-hover-seconds "${BASE1_SETTLED_HOVER_S:-15}" \
    2>&1 | tee "${flight_log}"
  status=${PIPESTATUS[0]}
  set -e
  (( status != 0 )) && overall=1
  for name in gazebo px4 agent settle px4_stability arm_init; do
    [[ -f "${runtime_dir}/${name}.log" ]] && \
      cp "${runtime_dir}/${name}.log" "${analysis_dir}/${label}_${name}.log"
  done
  echo "BASE1_ARM_STATE_END ${mode} status=${status}"
done

report="${analysis_dir}/base1_arm_state_${run_id}_comparison.json"
set +e
python3 "${workspace_dir}/scripts/analyze_base1_arm_state_hover.py" \
  "${logs[0]}" "${logs[1]}" --output "${report}"
analysis_status=$?
set -e
echo "BASE1_ARM_STATE_REPORT ${report}"
(( overall == 0 && analysis_status == 0 ))
