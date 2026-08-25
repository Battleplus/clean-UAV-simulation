#!/usr/bin/env bash
set -euo pipefail
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source install/setup.bash
set -u

# The acceptance must execute the current workspace, never an older installed
# entry point left by a previous run.
colcon build --symlink-install --packages-select drone_arm_sim px4_ros2_control \
  2>&1 | tee analysis/base1/directional_workspace_colcon_build.log
set +u
source install/setup.bash
set -u

envelope="${ARM_WORKSPACE_ENVELOPE:-analysis/base1/arm_workspace_envelope_4kg.json}"
plan="${ARM_DIRECTIONAL_PLAN:-analysis/base1/directional_workspace_flight_plan_4kg.json}"
log_file="${1:-analysis/base1/directional_workspace_flight_acceptance_4kg.log}"
exit_file="${log_file%.log}.exit"
telemetry_file="${log_file%.log}.telemetry.jsonl"
telemetry_node_log="${log_file%.log}.telemetry_recorder.log"
telemetry_report="${log_file%.log}.telemetry_report.json"
event_archive="${log_file%.log}.events"
runtime_telemetry_file="/tmp/my_drone_directional_telemetry_${$}.jsonl"
candidate_urdf="${ARM_CANDIDATE_URDF:-src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf}"
candidate_reference="${ARM_CANDIDATE_MOTION_REFERENCE:-src/drone_arm_sim/config/so101_motion_reference_4kg.json}"
candidate_config="${ARM_CANDIDATE_FLIGHT_CONFIG:-src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json}"
candidate_mass_kg="${ARM_CANDIDATE_MASS_KG:-4.0}"
gravity_torque_limit_nm="${ARM_CANDIDATE_GRAVITY_TORQUE_LIMIT_NM:-1.35}"
maximum_motor_delta_n="${ARM_CANDIDATE_MAXIMUM_MOTOR_DELTA_N:-1.60}"
backend_launcher="${ARM_CANDIDATE_BACKEND_LAUNCHER:-scripts/wsl_start_ros2_dds_debug_4kg.sh}"

# The flight plan inherits source hashes from this envelope.  Rebuild it for
# every acceptance run so a changed URDF/config can never pair with stale
# workspace evidence merely because an old JSON file still exists.
python3 scripts/scan_arm_workspace_4kg.py \
  --urdf "${candidate_urdf}" \
  --motion-reference "${candidate_reference}" \
  --flight-config "${candidate_config}" \
  --target-mass-kg "${candidate_mass_kg}" \
  --gravity-torque-limit-nm "${gravity_torque_limit_nm}" \
  --maximum-motor-delta-n "${maximum_motor_delta_n}" \
  --output "${envelope}"

python3 scripts/plan_directional_workspace_acceptance_4kg.py \
  --envelope "${envelope}" \
  --output "${plan}" \
  --urdf "${candidate_urdf}" \
  --motion-reference "${candidate_reference}" \
  --flight-config "${candidate_config}" \
  --target-mass-kg "${candidate_mass_kg}" \
  --gravity-torque-limit-nm "${gravity_torque_limit_nm}" \
  --maximum-motor-delta-n "${maximum_motor_delta_n}" \
  --duration "${ARM_DIRECTIONAL_REQUESTED_DURATION_S:-20}" \
  --hold "${ARM_DIRECTIONAL_HOLD_S:-3}" \
  --settle "${ARM_DIRECTIONAL_SETTLE_S:-2}" \
  --sample-count "${ARM_DIRECTIONAL_PREFLIGHT_SAMPLES:-41}" \
  --shoulder-pan-speed-cap-rad-s "${ARM_DIRECTIONAL_SHOULDER_PAN_SPEED_CAP_RAD_S:-0}"

timeout_s="$(python3 - "${plan}" "${ARM_DIRECTIONAL_ONLY:-}" <<'PY'
import json
import math
from pathlib import Path
import sys
plan = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
requested = [item.strip() for item in sys.argv[2].split(",") if item.strip()]
if requested:
    known = {str(leg["direction"]): leg for leg in plan["legs"]}
    unknown = [direction for direction in requested if direction not in known]
    if unknown:
        raise SystemExit("unknown ARM_DIRECTIONAL_ONLY: " + ",".join(unknown))
    selected = [known[direction] for direction in requested]
    motion_s = sum(
        float(leg["outward"]["effective_duration_s"])
        + float(leg["hold_s"])
        + float(leg["return"]["effective_duration_s"])
        + float(leg["settle_s"])
        for leg in selected
    )
else:
    motion_s = float(plan["total_motion_s"])
print(int(math.ceil(motion_s + 180.0)))
PY
)"

if [[ "${DIRECTIONAL_RESTART_BACKEND:-1}" == "1" ]]; then
  ENABLE_ARM_CONTROL=true \
  HEADLESS="${DIRECTIONAL_HEADLESS:-true}" \
  CLEAN_STALE_RUNTIME=1 \
  BASE1_AUTO_ARM_COMPENSATION=true \
  BASE1_ADAPTIVE_ENABLED=false \
    bash "${backend_launcher}" \
    2>&1 | tee analysis/base1/directional_workspace_backend.log
fi

mkdir -p "$(dirname "${log_file}")"
rm -f "${exit_file}" "${telemetry_file}" "${telemetry_node_log}" \
  "${telemetry_report}" "${event_archive}"
setsid "${workspace_dir}/scripts/run_with_cpu_role.sh" bulk \
  ros2 run drone_arm_sim acceptance_telemetry_recorder \
  --output "${runtime_telemetry_file}" --maximum-rate-hz 100 \
  >"${telemetry_node_log}" 2>&1 &
telemetry_pid=$!
cleanup_telemetry() {
  if kill -0 "${telemetry_pid}" 2>/dev/null; then
    kill -TERM -- "-${telemetry_pid}" 2>/dev/null || true
    wait "${telemetry_pid}" 2>/dev/null || true
  fi
}
trap cleanup_telemetry EXIT
sleep 1
if ! kill -0 "${telemetry_pid}" 2>/dev/null; then
  cat "${telemetry_node_log}" >&2 || true
  echo "acceptance telemetry recorder failed to start" >&2
  exit 1
fi
set +e
ARM_FLIGHT_PROFILE=directional_workspace_4kg \
ARM_DIRECTIONAL_PLAN="${plan}" \
SO101_KINEMATICS_URDF="${candidate_urdf}" \
SO101_MOTION_REFERENCE="${candidate_reference}" \
MY_DRONE_FLIGHT_CONFIG="${candidate_config}" \
ARM_FLIGHT_ACCEPT_HORIZONTAL_M=0.05 \
ARM_FLIGHT_ACCEPT_ALTITUDE_M=0.05 \
ARM_FLIGHT_ACCEPT_TILT_DEG=1.0 \
PX4_TRUTH_HOLD_ENABLED=true \
PX4_TRUTH_HOLD_XY_P="${PX4_TRUTH_HOLD_XY_P:-0.80}" \
PX4_TRUTH_HOLD_XY_D="${PX4_TRUTH_HOLD_XY_D:-0.0}" \
PX4_TRUTH_HOLD_Z_P="${PX4_TRUTH_HOLD_Z_P:-1.30}" \
PX4_TRUTH_HOLD_Z_D="${PX4_TRUTH_HOLD_Z_D:-0.45}" \
  python3 scripts/test_ros2_dds_arm_flight_pty.py --timeout "${timeout_s}" \
  2>&1 | tee "${log_file}"
result=${PIPESTATUS[0]}
set -e
cleanup_telemetry
trap - EXIT
if [[ -s "${runtime_telemetry_file}" ]]; then
  cp "${runtime_telemetry_file}" "${telemetry_file}"
  rm -f "${runtime_telemetry_file}"
fi
if [[ -f /tmp/my_drone_directional_workspace.events ]]; then
  cp /tmp/my_drone_directional_workspace.events "${event_archive}"
fi
if [[ -s "${telemetry_file}" && -s "${event_archive}" ]]; then
  ownership_analysis_args=()
  if [[ "${ARM_DIRECT_XY_OWNERSHIP:-false}" == "true" ]]; then
    ownership_analysis_args+=(--require-direct-xy-ownership)
  fi
  set +e
  python3 scripts/analyze_directional_telemetry.py \
    --telemetry "${telemetry_file}" \
    --events "${event_archive}" \
    --config "${candidate_config}" \
    --output "${telemetry_report}" \
    --expected-directions "${ARM_DIRECTIONAL_ONLY:-front,rear,left,right,up,down,front_left,front_right,rear_left,rear_right}" \
    "${ownership_analysis_args[@]}" \
    --xy-limit 0.05 --z-limit 0.05 --tilt-limit 1.0 \
    --minimum-rate-hz 50 --maximum-gap-s 0.20 --residual-limit 0.02 \
    2>&1 | tee -a "${log_file}"
  telemetry_result=${PIPESTATUS[0]}
  set -e
  if [[ "${telemetry_result}" -ne 0 ]]; then
    result=1
  fi
else
  echo "HIGH_RATE_DIRECTIONAL_ACCEPTANCE pass=false reason=missing_evidence" \
    | tee -a "${log_file}"
  result=1
fi
printf '%s\n' "${result}" > "${exit_file}"
exit "${result}"
