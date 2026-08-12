#!/usr/bin/env bash
# Deterministic paired flight test for the sensor-side arm disturbance observer.
# This script intentionally stops any active Gazebo/PX4 runtime, so an explicit
# confirmation environment variable is required.

set -uo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis"
runtime_dir="/tmp/my_drone_ros2_dds"
run_id="${1:-20260811}"
observer_gain="${ARM_DOB_AB_GAIN:-0.25}"
case_config="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
case_urdf="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
case_world="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
case_airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
backend_launcher="${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh"
flight_driver="${workspace_dir}/scripts/test_ros2_dds_arm_flight_pty.py"
observer_source="${workspace_dir}/src/drone_arm_sim/drone_arm_sim/arm_disturbance_observer.py"
motor_model_source="${workspace_dir}/src/drone_arm_sim/drone_arm_sim/gazebo_direct_motor_model.py"
coupling_monitor_source="${workspace_dir}/src/drone_arm_sim/drone_arm_sim/arm_coupling_monitor.py"
px4_binary="${PX4_DIR:-/home/asus/PX4-Autopilot}/build/px4_sitl_default/bin/px4"
if ! python3 -c \
  'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and 0.0 < value <= 0.5 else 1)' \
  "${observer_gain}"; then
  echo "REFUSED: ARM_DOB_AB_GAIN must be finite and in (0, 0.5]" >&2
  exit 66
fi
gain_tag="$(printf '%.2f' "${observer_gain}" | tr '.' 'p')"

# The backend launcher sources ROS in its own child shell.  Source the same
# environment here as well because the PTY flight test later launches `ros2`
# directly from this parent process.
set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH:-}"
set -u

if [[ "${RUN_DOB_AB_CONFIRM:-0}" != "1" ]]; then
  echo "REFUSED: this test stops the active GUI/runtime. Re-run with RUN_DOB_AB_CONFIRM=1." >&2
  exit 64
fi

mkdir -p "${analysis_dir}"

common_environment() {
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
  export ARM_TORQUE_FEEDFORWARD_ENABLED=false
  export ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0
  export ARM_DISTURBANCE_OBSERVER_GAIN="${observer_gain}"
  export ARM_DISTURBANCE_OBSERVER_MAX_TORQUE_NM=0.08
  export ARM_DISTURBANCE_OBSERVER_MAX_DELTA_N=1.0
}

copy_backend_logs() {
  local label="$1"
  for name in gazebo px4 agent settle px4_stability arm_init; do
    if [[ -f "${runtime_dir}/${name}.log" ]]; then
      cp "${runtime_dir}/${name}.log" "${analysis_dir}/${label}_${name}.log"
    fi
  done
}

run_case() {
  local label="$1"
  local observer_enabled="$2"
  local backend_log="${analysis_dir}/${label}_backend.log"
  local flight_log="${analysis_dir}/${label}_flight.log"

  common_environment
  export ARM_DISTURBANCE_OBSERVER_ENABLED="${observer_enabled}"

  if ! python3 "${workspace_dir}/scripts/write_arm_dob_case_manifest.py" \
    --output "${analysis_dir}/${label}_case_manifest.json" \
    --label "${label}" \
    --observer-enabled "${observer_enabled}" \
    --observer-gain "${observer_gain}" \
    --config "${case_config}" \
    --urdf "${case_urdf}" \
    --world "${case_world}" \
    --airframe "${case_airframe}" \
    --backend-launcher "${backend_launcher}" \
    --flight-driver "${flight_driver}" \
    --observer-source "${observer_source}" \
    --motor-model-source "${motor_model_source}" \
    --coupling-monitor-source "${coupling_monitor_source}" \
    --px4-binary "${px4_binary}" \
    >/dev/null; then
    echo "CASE_MANIFEST_FAIL label=${label}" >&2
    return 72
  fi
  echo "CASE_BEGIN label=${label} observer_enabled=${observer_enabled}"
  set +e
  bash "${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh" \
    2>&1 | tee "${backend_log}"
  local backend_status=${PIPESTATUS[0]}
  set -e
  if [[ ${backend_status} -ne 0 ]]; then
    copy_backend_logs "${label}"
    echo "CASE_BACKEND_FAIL label=${label} status=${backend_status}" >&2
    return "${backend_status}"
  fi

  if [[ "${observer_enabled}" == "true" ]]; then
    local observer_ready=0
    for _ in $(seq 1 20); do
      if grep -q "arm_disturbance_observer.*process has died\|arm_disturbance_observer.*unrecognized arguments" "${runtime_dir}/gazebo.log" 2>/dev/null; then
        copy_backend_logs "${label}"
        echo "CASE_OBSERVER_RUNTIME_FAIL label=${label} reason=process_died" >&2
        return 70
      fi
      if grep -q "ARM_DOB_STATE" "${runtime_dir}/gazebo.log" 2>/dev/null; then
        observer_ready=1
        break
      fi
      sleep 0.5
    done
    if [[ ${observer_ready} -ne 1 ]]; then
      copy_backend_logs "${label}"
      echo "CASE_OBSERVER_RUNTIME_FAIL label=${label} reason=no_diagnostic_marker" >&2
      return 71
    fi
  fi

  set +e
  python3 "${workspace_dir}/scripts/test_ros2_dds_arm_flight_pty.py" \
    --timeout 300 2>&1 | tee "${flight_log}"
  local flight_status=${PIPESTATUS[0]}
  set -e
  copy_backend_logs "${label}"
  echo "CASE_END label=${label} status=${flight_status}"
  return "${flight_status}"
}

off_label="arm_dob_ab_${run_id}_off"
on_label="arm_dob_ab_${run_id}_on_g${gain_tag}"
off_status=0
on_status=0
run_case "${off_label}" false || off_status=$?
run_case "${on_label}" true || on_status=$?

comparison_output="${analysis_dir}/arm_dob_ab_${run_id}_comparison.json"
set +e
python3 "${workspace_dir}/scripts/compare_arm_flight_ab.py" \
  --feedforward-off "${analysis_dir}/${off_label}_flight.log" \
  --feedforward-on "${analysis_dir}/${on_label}_flight.log" \
  --candidate-runtime-log "${analysis_dir}/${on_label}_gazebo.log" \
  --candidate-runtime-required-marker ARM_DOB_STATE \
  --off-case-manifest "${analysis_dir}/${off_label}_case_manifest.json" \
  --on-case-manifest "${analysis_dir}/${on_label}_case_manifest.json" \
  --required-presets demo_extended retracted \
  --horizontal-gate 0.15 \
  --altitude-gate 0.30 \
  --tilt-gate 3.0 \
  --arm-torque-gate 0.50 \
  --require-improvement \
  --output "${comparison_output}"
comparison_status=$?
set -e

echo "DOB_AB_RESULT off_status=${off_status} on_status=${on_status} comparison_status=${comparison_status}"
echo "DOB_AB_REPORT ${comparison_output}"
echo "NOTICE: headless ON case is still the active backend; restore the standard GUI after reviewing logs."

if [[ ${off_status} -ne 0 || ${on_status} -ne 0 ]]; then
  exit 2
fi
exit "${comparison_status}"
