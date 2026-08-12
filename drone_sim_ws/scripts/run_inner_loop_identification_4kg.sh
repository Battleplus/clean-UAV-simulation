#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis"
runtime_dir="/tmp/my_drone_ros2_dds"
tag="${1:-$(date +%Y%m%d_%H%M%S)}"
prefix="inner_loop_identification_4kg_${tag}"
raw_report="${analysis_dir}/${prefix}_raw.json"
loop_report="${analysis_dir}/${prefix}_loops.json"
flight_log="${analysis_dir}/${prefix}_flight.log"
source_airframe="${PROJECT_AIRFRAME_FILE:-${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg}"
tuning_decision="${analysis_dir}/${prefix}_tuning_decision.json"
candidate_airframe="${analysis_dir}/${prefix}_airframe_candidate_debug_4kg"
candidate_manifest="${analysis_dir}/${prefix}_airframe_candidate_manifest.json"

if [[ "${RUN_INNER_LOOP_ID_CONFIRM:-0}" != "1" ]]; then
  echo "DRY_RUN: set RUN_INNER_LOOP_ID_CONFIRM=1 to stop the manual GUI and arm the 4 kg diagnostic" >&2
  exit 64
fi

# A confirmation copied from an earlier shell must not silently terminate a
# later interactive flight.  Require a second, explicit takeover opt-in when
# either the Gazebo GUI or manual DDS keyboard controller is currently alive.
interactive_processes=()
if pgrep -f '[g]z sim gui' >/dev/null 2>&1; then
  interactive_processes+=("gazebo_gui")
fi
if pgrep -f '[d]ds_wasd_control' >/dev/null 2>&1; then
  interactive_processes+=("dds_wasd_control")
fi
if (( ${#interactive_processes[@]} > 0 )) && \
   [[ "${RUN_INNER_LOOP_ID_TAKEOVER:-0}" != "1" ]]; then
  printf 'INTERACTIVE_RUNTIME_ACTIVE: %s\n' "${interactive_processes[*]}" >&2
  echo "REFUSED: V3 identification needs an exclusive backend." >&2
  echo "Close the GUI/WASD session, or explicitly set RUN_INNER_LOOP_ID_TAKEOVER=1." >&2
  exit 65
fi

mkdir -p "${analysis_dir}"
set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

echo "INNER_LOOP_ID_BACKEND_START tag=${tag}"
HEADLESS=true \
ENABLE_ARM_CONTROL=true \
CLEAN_STALE_RUNTIME=1 \
PX4_FRESH_WORKDIR=1 \
MODEL_SETTLE_S=8 \
bash "${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh"

set +e
ros2 run px4_ros2_control inner_loop_identification \
  --output "${raw_report}" \
  --confirm-4kg-debug 2>&1 | tee "${flight_log}"
flight_status=${PIPESTATUS[0]}
set -e
if [[ -f "${raw_report}" ]] && ! python3 -c \
  'import json,sys; raise SystemExit(0 if json.load(open(sys.argv[1], encoding="utf-8"))["result"] == "INNER_LOOP_IDENTIFICATION_PASS" else 1)' \
  "${raw_report}"; then
  flight_status=1
fi

for name in agent gazebo px4 arm_init settle px4_stability; do
  if [[ -f "${runtime_dir}/${name}.log" ]]; then
    cp "${runtime_dir}/${name}.log" "${analysis_dir}/${prefix}_${name}.log"
  fi
done

ulog=""
if [[ -f "${runtime_dir}/px4_workdir.path" ]]; then
  px4_workdir="$(cat "${runtime_dir}/px4_workdir.path")"
  ulog="$(find "${px4_workdir}/log" -type f -name '*.ulg' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"
fi

analysis_status=1
decision_status=1
candidate_status=0
if [[ -n "${ulog}" && -f "${ulog}" ]]; then
  /home/asus/PX4-Autopilot/.venv/bin/python \
    "${workspace_dir}/scripts/analyze_control_loops_ulog.py" \
    "${ulog}" --dedicated-inner-loop-step-protocol --output "${loop_report}"
  analysis_status=0
  echo "INNER_LOOP_ID_ULOG ${ulog}"

  decision_command=(
    python3 "${workspace_dir}/scripts/derive_px4_tuning_candidate.py"
    "${loop_report}" "${source_airframe}"
    --output "${tuning_decision}"
  )
  decision_status=0
  if [[ -n "${VELOCITY_IDENTIFICATION_EVIDENCE:-}" ]]; then
    if [[ ! -f "${VELOCITY_IDENTIFICATION_EVIDENCE}" ]]; then
      echo "VELOCITY_IDENTIFICATION_EVIDENCE_MISSING ${VELOCITY_IDENTIFICATION_EVIDENCE}" >&2
      decision_status=1
    else
      decision_command+=(--velocity-evidence "${VELOCITY_IDENTIFICATION_EVIDENCE}")
    fi
  fi
  if [[ ${decision_status} -eq 0 ]]; then
    "${decision_command[@]}"
    decision_status=$?
  fi

  if [[ ${decision_status} -eq 0 ]] && python3 -c \
    'import json,sys; raise SystemExit(0 if json.load(open(sys.argv[1], encoding="utf-8"))["decision"] == "BOUNDED_CANDIDATE_REQUIRES_RETEST" else 1)' \
    "${tuning_decision}"; then
    if ! python3 "${workspace_dir}/scripts/build_px4_tuning_candidate_airframe.py" \
      "${source_airframe}" "${tuning_decision}" "${candidate_airframe}" \
      --manifest "${candidate_manifest}"; then
      candidate_status=1
    fi
  else
    echo "INNER_LOOP_ID_NO_PARAMETER_CANDIDATE decision_report=${tuning_decision}"
  fi
else
  echo "INNER_LOOP_ID_ULOG_MISSING" >&2
fi

echo "INNER_LOOP_ID_RESULT flight_status=${flight_status} analysis_status=${analysis_status} decision_status=${decision_status} candidate_status=${candidate_status}"
echo "INNER_LOOP_ID_RAW_REPORT ${raw_report}"
echo "INNER_LOOP_ID_LOOP_REPORT ${loop_report}"
echo "INNER_LOOP_ID_TUNING_DECISION ${tuning_decision}"
if [[ -f "${candidate_airframe}" ]]; then
  echo "INNER_LOOP_ID_CANDIDATE_AIRFRAME ${candidate_airframe}"
  echo "NOTICE: candidate is disposable and not applied; rerun the same protocol with PROJECT_AIRFRAME_FILE set to this file."
fi
echo "NOTICE: headless diagnostic backend remains active; restore the standard GUI after review."

if [[ ${flight_status} -ne 0 || ${analysis_status} -ne 0 || ${decision_status} -ne 0 || ${candidate_status} -ne 0 ]]; then
  exit 1
fi
