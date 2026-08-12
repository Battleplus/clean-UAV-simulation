#!/usr/bin/env bash
# Deterministic Base 1 4 kg baseline. Every compensation path is forced off.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis/base1"
runtime_dir="/tmp/my_drone_ros2_dds"
run_id="${1:-$(date +%Y%m%d_%H%M%S)}"
repeat_count="${BASE1_REPEAT_COUNT:-3}"

if [[ "${RUN_BASE1_BASELINE_CONFIRM:-0}" != "1" ]]; then
  echo "REFUSED: this test replaces the active Gazebo/PX4 runtime." >&2
  echo "Re-run with RUN_BASE1_BASELINE_CONFIRM=1." >&2
  exit 64
fi
if ! [[ "${repeat_count}" =~ ^[1-9][0-9]*$ ]] || (( repeat_count > 5 )); then
  echo "REFUSED: BASE1_REPEAT_COUNT must be an integer in [1,5]." >&2
  exit 65
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

mkdir -p "${analysis_dir}"
python3 "${workspace_dir}/scripts/build_base1_freeze_manifest.py" \
  --output "${workspace_dir}/baselines/Base_1_flight_freeze.json"

flight_logs=()
overall=0
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
px4_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/4027_gz_my_drone_octorotor_debug_4kg"
for index in $(seq 1 "${repeat_count}"); do
  label="base1_no_comp_${run_id}_run${index}"
  backend_log="${analysis_dir}/${label}_backend.log"
  flight_log="${analysis_dir}/${label}_flight.log"
  env_log="${analysis_dir}/${label}_environment.txt"

  export HEADLESS=true
  export ENABLE_ARM_CONTROL=true
  export CLEAN_STALE_RUNTIME=1
  export PX4_FRESH_WORKDIR=1
  export GZ_RANDOM_SEED=4027
  export MODEL_SETTLE_S=8
  export MODEL_SETTLE_HOLD_S=2
  export PX4_READY_SETTLE_S=5
  export PX4_READY_STABLE_HOLD_S=5
  export ARM_FLIGHT_GROUND_STABLE_HOLD_S=5
  export ARM_FLIGHT_PROFILE=full_extend_slow_4kg
  export ARM_FEEDFORWARD_ENABLED=false
  export ARM_TORQUE_FEEDFORWARD_ENABLED=false
  export ARM_REACTION_TORQUE_FEEDFORWARD_GAIN=0.0
  export ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0
  export ARM_DISTURBANCE_OBSERVER_ENABLED=false
  # This is a test-harness-only landing tolerance. It is used after Gazebo
  # truth is already on the ground because EKF local-z can retain ~0.15 m of
  # offset. It does not alter flight, hover, arm motion, or Base 1 defaults.
  export PX4_TOUCHDOWN_DISARM_ENABLED=true
  export PX4_TOUCHDOWN_DISARM_HEIGHT_M="${BASE1_TEST_TOUCHDOWN_DISARM_HEIGHT_M:-0.20}"
  export PX4_TOUCHDOWN_DISARM_HOLD_S=0.5

  export AIRFRAME_ID=4027
  export PROJECT_AIRFRAME_FILE="${airframe}"
  export ROBOT_FILE="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
  export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
  export MY_DRONE_WORLD="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
  export ARM_COUPLING_TARGET_MASS_KG=4.0
  export BATTERY_DYNAMICS_ENABLED=false
  export ENABLE_SENSOR_DELAY=true
  export IMU_DELAY_MS=0 MAG_DELAY_MS=0 BARO_DELAY_MS=0 NAVSAT_DELAY_MS=0
  export REACTION_MOMENT_RATIO_M=0.005
  export PX4_WASD_VERTICAL_SPEED_M_S=0.15
  export SPAWN_Z=0.289
  cp "${airframe}" "${px4_airframe}"
  chmod +x "${px4_airframe}"

  {
    echo "base1_ref=base-1"
    echo "base1_commit=b340ed6"
    echo "scope=4kg_only"
    echo "profile=${ARM_FLIGHT_PROFILE}"
    echo "seed=${GZ_RANDOM_SEED}"
    echo "ARM_FEEDFORWARD_ENABLED=${ARM_FEEDFORWARD_ENABLED}"
    echo "ARM_TORQUE_FEEDFORWARD_ENABLED=${ARM_TORQUE_FEEDFORWARD_ENABLED}"
    echo "ARM_REACTION_TORQUE_FEEDFORWARD_GAIN=${ARM_REACTION_TORQUE_FEEDFORWARD_GAIN}"
    echo "ARM_STATIC_COM_FEEDFORWARD_GAIN=${ARM_STATIC_COM_FEEDFORWARD_GAIN}"
    echo "ARM_DISTURBANCE_OBSERVER_ENABLED=${ARM_DISTURBANCE_OBSERVER_ENABLED}"
    echo "test_only_touchdown_disarm_height_m=${PX4_TOUCHDOWN_DISARM_HEIGHT_M}"
  } >"${env_log}"

  echo "BASE1_CASE_BEGIN ${label}"
  if ! bash "${workspace_dir}/scripts/wsl_start_ros2_dds_noarm.sh" \
      2>&1 | tee "${backend_log}"; then
    echo "BASE1_BACKEND_FAIL ${label}" >&2
    overall=1
    continue
  fi

  set +e
  python3 "${workspace_dir}/scripts/test_ros2_dds_arm_flight_pty.py" \
    --timeout "${BASE1_DRIVER_TIMEOUT_S:-360}" 2>&1 | tee "${flight_log}"
  flight_status=${PIPESTATUS[0]}
  set -e
  flight_logs+=("${flight_log}")

  for name in gazebo px4 agent settle px4_stability arm_init; do
    if [[ -f "${runtime_dir}/${name}.log" ]]; then
      cp "${runtime_dir}/${name}.log" "${analysis_dir}/${label}_${name}.log"
    fi
  done
  if (( flight_status != 0 )); then
    overall=1
  fi
  echo "BASE1_CASE_END ${label} status=${flight_status}"
done

if (( ${#flight_logs[@]} == 0 )); then
  echo "BASE1_BASELINE_FAIL no flight logs" >&2
  exit 2
fi

summary="${analysis_dir}/base1_no_comp_${run_id}_summary.json"
set +e
python3 "${workspace_dir}/scripts/analyze_base1_no_compensation.py" \
  "${flight_logs[@]}" --output "${summary}"
summary_status=$?
set -e
echo "BASE1_BASELINE_REPORT ${summary}"

if (( overall != 0 || summary_status != 0 || ${#flight_logs[@]} != repeat_count )); then
  exit 1
fi
echo "BASE1_NO_COMPENSATION_BASELINE_PASS runs=${repeat_count}"
