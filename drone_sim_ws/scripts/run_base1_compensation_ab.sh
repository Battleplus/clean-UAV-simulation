#!/usr/bin/env bash
# Strict paired A/B for one Base 1 arm-compensation channel.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis/base1"
runtime_dir="/tmp/my_drone_ros2_dds"
channel="${1:-}"
gain="${2:-0.05}"
run_id="${3:-$(date +%Y%m%d_%H%M%S)}"

if [[ "${RUN_BASE1_COMP_AB_CONFIRM:-0}" != "1" ]]; then
  echo "REFUSED: paired A/B replaces the active Gazebo/PX4 runtime." >&2
  echo "Re-run with RUN_BASE1_COMP_AB_CONFIRM=1." >&2
  exit 64
fi
case "${channel}" in
  reaction_force|gravity_torque|reaction_torque) ;;
  *) echo "REFUSED: invalid channel ${channel}" >&2; exit 65 ;;
esac
if [[ "${gain}" != "0.05" && "${gain}" != "0.10" ]]; then
  echo "REFUSED: gain must be exactly 0.05 or 0.10" >&2
  exit 66
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
mkdir -p "${analysis_dir}"

airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
off_flight=""
on_flight=""
on_reallocator=""
overall=0

for mode in off on; do
  label="base1_comp_${channel}_g${gain/./p}_${run_id}_${mode}"
  backend_log="${analysis_dir}/${label}_backend.log"
  overlay_log="${analysis_dir}/${label}_overlay.log"
  flight_log="${analysis_dir}/${label}_flight.log"
  reallocator_log="${analysis_dir}/${label}_reallocator.log"

  export HEADLESS=true ENABLE_ARM_CONTROL=true CLEAN_STALE_RUNTIME=1
  export PX4_FRESH_WORKDIR=1 GZ_RANDOM_SEED=4027
  export MODEL_SETTLE_S=8 MODEL_SETTLE_HOLD_S=2
  export PX4_READY_SETTLE_S=20 PX4_READY_STABLE_HOLD_S=10
  export PX4_READY_STABLE_TIMEOUT_S=180
  export ARM_FLIGHT_PROFILE=full_extend_slow_4kg
  export ARM_FEEDFORWARD_ENABLED=false ARM_TORQUE_FEEDFORWARD_ENABLED=false
  export ARM_STATIC_COM_FEEDFORWARD_GAIN=0 ARM_DISTURBANCE_OBSERVER_ENABLED=false
  export PX4_TOUCHDOWN_DISARM_ENABLED=true
  export PX4_TOUCHDOWN_DISARM_HEIGHT_M=0.20 PX4_TOUCHDOWN_DISARM_HOLD_S=0.5
  export AIRFRAME_ID=4027 PROJECT_AIRFRAME_FILE="${airframe}"
  export ROBOT_FILE="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
  export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
  export MY_DRONE_WORLD="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
  export ARM_COUPLING_TARGET_MASS_KG=4.0 BATTERY_DYNAMICS_ENABLED=false
  export ENABLE_SENSOR_DELAY=true IMU_DELAY_MS=0 MAG_DELAY_MS=0 BARO_DELAY_MS=0 NAVSAT_DELAY_MS=0
  export REACTION_MOMENT_RATIO_M=0.005 PX4_WASD_VERTICAL_SPEED_M_S=0.15 SPAWN_Z=0.289
  export BASE1_COMPENSATION_ENABLED=true
  export BASE1_REACTION_FORCE_GAIN=0 BASE1_GRAVITY_TORQUE_GAIN=0 BASE1_REACTION_TORQUE_GAIN=0
  if [[ "${mode}" == "on" ]]; then
    case "${channel}" in
      reaction_force) export BASE1_REACTION_FORCE_GAIN="${gain}" ;;
      gravity_torque) export BASE1_GRAVITY_TORQUE_GAIN="${gain}" ;;
      reaction_torque) export BASE1_REACTION_TORQUE_GAIN="${gain}" ;;
    esac
  fi

  echo "BASE1_COMP_AB_BEGIN mode=${mode} channel=${channel} gain=${gain}"
  if ! bash "${workspace_dir}/scripts/wsl_start_ros2_dds_noarm.sh" \
      2>&1 | tee "${backend_log}"; then
    overall=1
    break
  fi
  if ! bash "${workspace_dir}/scripts/activate_base1_wrench_reallocator_overlay.sh" \
      2>&1 | tee "${overlay_log}"; then
    overall=1
    break
  fi
  # Overlay startup intentionally happens while disarmed and can take tens of
  # seconds on a cold ROS graph.  Re-run the unchanged strict PX4 stability
  # gate afterwards so no pair can arm from a delayed EKF reset.
  if ! python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
      --horizontal 0.10 --vertical 0.08 --hold 10 --timeout 180 \
      >"${analysis_dir}/${label}_post_overlay_stability.log" 2>&1; then
    echo "BASE1_COMP_AB_POST_OVERLAY_STABILITY_FAIL mode=${mode}" >&2
    overall=1
    break
  fi
  set +e
  python3 "${workspace_dir}/scripts/test_ros2_dds_arm_flight_pty.py" \
    --timeout "${BASE1_COMP_DRIVER_TIMEOUT_S:-420}" 2>&1 | tee "${flight_log}"
  flight_status=${PIPESTATUS[0]}
  set -e
  cp "${runtime_dir}/base1_reallocator.log" "${reallocator_log}"
  [[ "${mode}" == "off" ]] && off_flight="${flight_log}"
  if [[ "${mode}" == "on" ]]; then
    on_flight="${flight_log}"
    on_reallocator="${reallocator_log}"
  fi
  echo "BASE1_COMP_AB_END mode=${mode} status=${flight_status}"
  if (( flight_status != 0 )); then
    overall=1
    echo "BASE1_COMP_AB_ABORT mode=${mode}: refusing to start the next side after a failed flight" >&2
    break
  fi
done

if [[ -z "${off_flight}" || -z "${on_flight}" || -z "${on_reallocator}" ]]; then
  echo "BASE1_COMP_AB_FAIL incomplete pair" >&2
  exit 2
fi
report="${analysis_dir}/base1_comp_${channel}_g${gain/./p}_${run_id}_comparison.json"
set +e
python3 "${workspace_dir}/scripts/analyze_base1_compensation_ab.py" \
  --off-flight "${off_flight}" --on-flight "${on_flight}" \
  --on-reallocator "${on_reallocator}" --channel "${channel}" \
  --gain "${gain}" --output "${report}"
analysis_status=$?
set -e
echo "BASE1_COMP_AB_REPORT ${report}"
(( overall == 0 && analysis_status == 0 ))
