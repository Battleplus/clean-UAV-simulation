#!/usr/bin/env bash
# Strict, deterministic Base1 slow-adaptive A/B campaign.
#
# This intentionally stops/replaces the active runtime twelve times.  It is
# opt-in and initializes a NOT_RUN result before asking for confirmation, so a
# prepared campaign can never be mistaken for flight evidence.

set -uo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
campaign_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
campaign_dir="${workspace_dir}/analysis/slow_adaptive_ab_${campaign_id}"
runtime_dir="/tmp/my_drone_ros2_dds"
nominal_urdf="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
payload_urdf="${campaign_dir}/physical_gripper_payload_10g_unmodeled.urdf"
manifest="${campaign_dir}/campaign_manifest.json"
result="${campaign_dir}/campaign_result.json"
mkdir -p "${campaign_dir}"

python3 "${workspace_dir}/scripts/build_payload_urdf.py" \
  --source "${nominal_urdf}" \
  --output "${payload_urdf}" \
  --mass-kg 0.010 \
  --size-m 0.02 \
  --offset 0.08 0 0 \
  --attachment-mode merged
python3 "${workspace_dir}/scripts/analyze_slow_adaptive_ab_4kg.py" init \
  --output "${manifest}" \
  --campaign-id "${campaign_id}" \
  --workspace "${workspace_dir}" \
  --payload-urdf "${payload_urdf}"
python3 "${workspace_dir}/scripts/analyze_slow_adaptive_ab_4kg.py" analyze \
  --manifest "${manifest}" --campaign-dir "${campaign_dir}" --output "${result}" \
  >/dev/null 2>&1 || true

if [[ "${RUN_SLOW_ADAPTIVE_AB_CONFIRM:-0}" != "1" ]]; then
  echo "REFUSED: campaign is initialized as NOT_RUN; set RUN_SLOW_ADAPTIVE_AB_CONFIRM=1 to stop the GUI and run 12 flights." >&2
  echo "ADAPTIVE_AB_NOT_RUN result=${result}"
  exit 64
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH:-}"
set -u

# Never run a campaign against a stale installed Python entry point.  This was
# the cause of earlier sessions opening old arm/control code after source edits.
colcon build --symlink-install --packages-select drone_arm_sim px4_ros2_control \
  2>&1 | tee "${campaign_dir}/colcon_build.log"
build_status=${PIPESTATUS[0]}
if [[ ${build_status} -ne 0 ]]; then
  echo "ADAPTIVE_AB_NOT_RUN reason=colcon_build_failed status=${build_status}" >&2
  exit "${build_status}"
fi
set +u
source "${workspace_dir}/install/setup.bash"
set -u

common_environment() {
  export HEADLESS=true
  export ENABLE_ARM_CONTROL=true
  export CLEAN_STALE_RUNTIME=1
  export PX4_FRESH_WORKDIR=1
  export GZ_RANDOM_SEED=4027
  export AIRFRAME_ID=4027
  export PROJECT_AIRFRAME_FILE="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
  export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
  export SO101_MOTION_REFERENCE="${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference_4kg.json"
  export SO101_KINEMATICS_URDF="${nominal_urdf}"
  export SPAWN_Z=0.289
  export REACTION_MOMENT_RATIO_M=0.005
  export MODEL_SETTLE_S=8
  export MODEL_SETTLE_HOLD_S=2
  export PX4_READY_SETTLE_S=5
  export PX4_READY_STABLE_HOLD_S=5
  export ARM_FLIGHT_GROUND_STABLE_HOLD_S=5
  export ARM_FLIGHT_PROFILE=nose_forward_straight_4kg
  export ARM_NOSE_FORWARD_EXTENSION_DURATION_S=90
  export ARM_NOSE_FORWARD_HOLD_S=8
  export ARM_NOSE_FORWARD_RETURN_DURATION_S=120
  export ARM_FLIGHT_ACCEPT_HORIZONTAL_M=0.05
  export ARM_FLIGHT_ACCEPT_ALTITUDE_M=0.05
  export ARM_FLIGHT_ACCEPT_TILT_DEG=1.0
  export PX4_TRUTH_HOLD_ENABLED=true
  export PX4_TRUTH_HOLD_XY_P=0.80
  export PX4_TRUTH_HOLD_XY_D=0.0
  export PX4_TRUTH_HOLD_Z_P=1.30
  export PX4_TRUTH_HOLD_Z_D=0.45
  export PX4_TRUTH_HOLD_VELOCITY_FILTER_TAU_S=0.0
  export PX4_TRUTH_HOLD_XY_MAX_M_S=0.08
  export PX4_TRUTH_HOLD_Z_MAX_M_S=0.12
  export PX4_TRUTH_HOLD_POSITION_GAIN=1.0
  export PX4_TRUTH_HOLD_POSITION_XY_MAX_M=0.15
  export PX4_TRUTH_HOLD_POSITION_Z_MAX_M=0.12
  export ARM_TORQUE_FEEDFORWARD_ENABLED=false
  export ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0
  export ARM_DISTURBANCE_OBSERVER_ENABLED=false
  export BASE1_AUTO_ARM_COMPENSATION=true
  export BASE1_REACTION_FORCE_GAIN=1.0
  export BASE1_REACTION_TORQUE_GAIN=1.0
  export BASE1_GRAVITY_TORQUE_GAIN=1.0
  export BASE1_COMP_FORCE_LIMIT_N=1.0
  export BASE1_REACTION_TORQUE_LIMIT_NM=0.10
  export BASE1_COMP_FORCE_SLEW_N_S=1.0
  export BASE1_COMP_TORQUE_SLEW_NM_S=0.10
  export BASE1_COMP_MIN_HEADROOM_N=0.25
  export BASE1_COMP_MAX_RESIDUAL_NORM=0.02
  export BASE1_POSITION_FEEDBACK_ENABLED=false
  export BASE1_ADAPTIVE_BASELINE_HOLD_S=5.0
  export BASE1_ADAPTIVE_TIME_CONSTANT_S=15.0
  export BASE1_ADAPTIVE_LEAK_TIME_CONSTANT_S=60.0
  export BASE1_ADAPTIVE_WARMUP_S=3.0
  export BASE1_ADAPTIVE_FORCE_DEADBAND_N=0.02
  export BASE1_ADAPTIVE_TORQUE_DEADBAND_NM=0.005
  export BASE1_ADAPTIVE_HORIZONTAL_FORCE_LIMIT_N=0.06
  export BASE1_ADAPTIVE_VERTICAL_FORCE_LIMIT_N=0.04
  export BASE1_ADAPTIVE_TORQUE_LIMIT_NM=0.02
  export BASE1_ADAPTIVE_HORIZONTAL_FORCE_RATE_N_S=0.005
  export BASE1_ADAPTIVE_VERTICAL_FORCE_RATE_N_S=0.003
  export BASE1_ADAPTIVE_TORQUE_RATE_NM_S=0.002
  export BASE1_ADAPTIVE_JOINT_VELOCITY_LIMIT_RAD_S=0.005
  export BASE1_ADAPTIVE_JOINT_ACCELERATION_LIMIT_RAD_S2=0.02
  export BASE1_ADAPTIVE_BODY_SPEED_LIMIT_M_S=0.03
  export BASE1_ADAPTIVE_ANGULAR_RATE_LIMIT_RAD_S=0.008726646
  export BASE1_ADAPTIVE_UPDATE_RESIDUAL_LIMIT=0.005
  # The physical payload cases change ROBOT_FILE only.  The Base1 read-only
  # estimator overlay intentionally remains fixed to the nominal URDF and a
  # zero payload argument, proving that the 10 g load is unmodelled.
  export ARM_COUPLING_TARGET_MASS_KG=4.0
  export ARM_PAYLOAD_MASS_KG=0.0
}

copy_case_logs() {
  local label="$1"
  local name
  for name in gazebo px4 agent settle px4_stability arm_init base1_estimator base1_reallocator base1_overlay_motor; do
    if [[ -f "${runtime_dir}/${name}.log" ]]; then
      cp "${runtime_dir}/${name}.log" "${campaign_dir}/${label}_${name}.log"
    fi
  done
  if [[ -f "${runtime_dir}/px4_workdir.path" ]]; then
    cp "${runtime_dir}/px4_workdir.path" "${campaign_dir}/${label}_px4_workdir.path"
  fi
}

run_case() {
  local group="$1"
  local pair="$2"
  local mode="$3"
  local label="adaptive_ab_${campaign_id}_${group}_p${pair}_${mode}"
  local backend_log="${campaign_dir}/${label}_backend.log"
  local flight_log="${campaign_dir}/${label}_flight.log"
  common_environment
  if [[ "${group}" == "payload_10g_unmodeled" ]]; then
    export ROBOT_FILE="${payload_urdf}"
  else
    export ROBOT_FILE="${nominal_urdf}"
  fi
  if [[ "${mode}" == "on" ]]; then
    export BASE1_ADAPTIVE_ENABLED=true
  else
    export BASE1_ADAPTIVE_ENABLED=false
  fi
  echo "CASE_BEGIN label=${label} seed=${GZ_RANDOM_SEED} fresh_px4=${PX4_FRESH_WORKDIR} physical_urdf=${ROBOT_FILE} estimator_urdf=${nominal_urdf} estimator_payload_kg=0.0"
  bash "${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh" 2>&1 | tee "${backend_log}"
  local backend_status=${PIPESTATUS[0]}
  if [[ ${backend_status} -eq 0 ]]; then
    python3 "${workspace_dir}/scripts/test_ros2_dds_arm_flight_pty.py" \
      --timeout 340 2>&1 | tee "${flight_log}"
    local flight_status=${PIPESTATUS[0]}
  else
    local flight_status=99
  fi
  copy_case_logs "${label}"
  echo "CASE_END label=${label} backend_status=${backend_status} flight_status=${flight_status}"
  [[ ${backend_status} -eq 0 && ${flight_status} -eq 0 ]]
}

campaign_failures=0
exec > >(tee -a "${campaign_dir}/campaign_runner.log") 2>&1
for group in nominal payload_10g_unmodeled; do
  for pair in 1 2 3; do
    run_case "${group}" "${pair}" off || campaign_failures=$((campaign_failures + 1))
    run_case "${group}" "${pair}" on || campaign_failures=$((campaign_failures + 1))
  done
done

python3 "${workspace_dir}/scripts/analyze_slow_adaptive_ab_4kg.py" analyze \
  --manifest "${manifest}" --campaign-dir "${campaign_dir}" --output "${result}"
analysis_status=$?
echo "ADAPTIVE_AB_COMPLETE flight_failures=${campaign_failures} analysis_status=${analysis_status} result=${result}"
if [[ ${campaign_failures} -ne 0 ]]; then
  exit 2
fi
exit "${analysis_status}"
