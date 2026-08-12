#!/usr/bin/env bash
# Inside-out PX4 identification/tuning gate for the disposable 4 kg model.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis"
tag="${1:-$(date +%Y%m%d_%H%M%S)}"
inner_tag="${tag}_inner"
velocity_tag="${tag}_velocity"
airframe="${PROJECT_AIRFRAME_FILE:-${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg}"
inner_loops="${analysis_dir}/inner_loop_identification_4kg_${inner_tag}_loops.json"
inner_decision="${analysis_dir}/inner_loop_identification_4kg_${inner_tag}_tuning_decision.json"
velocity_report="${analysis_dir}/velocity_identification_4kg_${velocity_tag}.json"
combined_decision="${analysis_dir}/layered_tuning_decision_4kg_${tag}.json"
candidate_airframe="${analysis_dir}/layered_tuning_candidate_4kg_${tag}_debug_4kg"
candidate_manifest="${analysis_dir}/layered_tuning_candidate_4kg_${tag}_manifest.json"

if [[ "${RUN_LAYERED_ID_CONFIRM:-0}" != "1" ]]; then
  echo "DRY_RUN: set RUN_LAYERED_ID_CONFIRM=1 to run armed 4 kg identification" >&2
  exit 64
fi

interactive_processes=()
if pgrep -f '[g]z sim gui' >/dev/null 2>&1; then
  interactive_processes+=("gazebo_gui")
fi
if pgrep -f '[d]ds_wasd_control' >/dev/null 2>&1; then
  interactive_processes+=("dds_wasd_control")
fi
if (( ${#interactive_processes[@]} > 0 )) && \
   [[ "${RUN_LAYERED_ID_TAKEOVER:-0}" != "1" ]]; then
  printf 'INTERACTIVE_RUNTIME_ACTIVE: %s\n' "${interactive_processes[*]}" >&2
  echo "REFUSED: layered identification needs an exclusive backend." >&2
  exit 65
fi

echo "LAYERED_ID_BEGIN tag=${tag} source_airframe=${airframe}"
RUN_INNER_LOOP_ID_CONFIRM=1 \
RUN_INNER_LOOP_ID_TAKEOVER=1 \
PROJECT_AIRFRAME_FILE="${airframe}" \
bash "${workspace_dir}/scripts/run_inner_loop_identification_4kg.sh" "${inner_tag}"

inner_active_layer="$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("active_layer") or "NONE")' \
  "${inner_decision}")"
inner_result="$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["decision"])' \
  "${inner_decision}")"
echo "LAYERED_ID_INNER_DECISION decision=${inner_result} active_layer=${inner_active_layer}"

# Velocity tuning is forbidden until every inner layer is complete and inside
# its gate.  The expected inner-only outcome is a HOLD at velocity because no
# independent 10 Hz velocity evidence has been supplied yet.
if [[ "${inner_active_layer}" != "velocity" ]]; then
  echo "LAYERED_ID_STOP_INNER_LAYER decision=${inner_result} active_layer=${inner_active_layer}"
  exit 2
fi

RUN_VELOCITY_ID_CONFIRM=1 \
RUN_VELOCITY_ID_TAKEOVER=1 \
PROJECT_AIRFRAME_FILE="${airframe}" \
bash "${workspace_dir}/scripts/run_velocity_identification_4kg.sh" "${velocity_tag}"

python3 "${workspace_dir}/scripts/derive_px4_tuning_candidate.py" \
  "${inner_loops}" "${airframe}" \
  --velocity-evidence "${velocity_report}" \
  --output "${combined_decision}"

combined_result="$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["decision"])' \
  "${combined_decision}")"
if [[ "${combined_result}" == "BOUNDED_CANDIDATE_REQUIRES_RETEST" ]]; then
  python3 "${workspace_dir}/scripts/build_px4_tuning_candidate_airframe.py" \
    "${airframe}" "${combined_decision}" "${candidate_airframe}" \
    --manifest "${candidate_manifest}"
  echo "LAYERED_ID_CANDIDATE ${candidate_airframe}"
  echo "LAYERED_ID_RETEST_REQUIRED use PROJECT_AIRFRAME_FILE=${candidate_airframe}"
elif [[ "${combined_result}" == "ALL_LAYERS_PASS_NO_PARAMETER_CHANGE" ]]; then
  echo "LAYERED_ID_ALL_LAYERS_PASS"
else
  echo "LAYERED_ID_HOLD decision=${combined_result}"
  exit 3
fi

echo "LAYERED_ID_DECISION ${combined_decision}"
