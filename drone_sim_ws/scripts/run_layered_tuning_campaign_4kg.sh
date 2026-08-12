#!/usr/bin/env bash
# Finite, evidence-gated retest campaign for disposable 4 kg PX4 candidates.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${LAYERED_TUNING_ANALYSIS_DIR:-${workspace_dir}/analysis}"
tag="${1:-$(date +%Y%m%d_%H%M%S)}"
current_airframe="${PROJECT_AIRFRAME_FILE:-${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg}"
max_iterations="${LAYERED_TUNING_MAX_ITERATIONS:-4}"
layered_runner="${LAYERED_ID_RUNNER:-${workspace_dir}/scripts/run_layered_identification_4kg.sh}"

if [[ "${RUN_LAYERED_TUNING_CONFIRM:-0}" != "1" ]]; then
  echo "DRY_RUN: set RUN_LAYERED_TUNING_CONFIRM=1 to run the armed 4 kg tuning campaign" >&2
  exit 64
fi
if ! [[ "${max_iterations}" =~ ^[1-9][0-9]*$ ]] || (( max_iterations > 6 )); then
  echo "INVALID_MAX_ITERATIONS: expected integer 1..6, got ${max_iterations}" >&2
  exit 66
fi

interactive_processes=()
if pgrep -f '[g]z sim gui' >/dev/null 2>&1; then
  interactive_processes+=("gazebo_gui")
fi
if pgrep -f '[d]ds_wasd_control' >/dev/null 2>&1; then
  interactive_processes+=("dds_wasd_control")
fi
if (( ${#interactive_processes[@]} > 0 )) && \
   [[ "${RUN_LAYERED_TUNING_TAKEOVER:-0}" != "1" ]]; then
  printf 'INTERACTIVE_RUNTIME_ACTIVE: %s\n' "${interactive_processes[*]}" >&2
  echo "REFUSED: layered tuning campaign needs an exclusive backend." >&2
  exit 65
fi

if [[ ! -f "${current_airframe}" ]]; then
  echo "SOURCE_AIRFRAME_MISSING ${current_airframe}" >&2
  exit 67
fi
if [[ ! -f "${layered_runner}" ]]; then
  echo "LAYERED_ID_RUNNER_MISSING ${layered_runner}" >&2
  exit 69
fi
if [[ "$(basename "${current_airframe}")" != *debug_4kg* ]]; then
  echo "REFUSED_NON_DEBUG_AIRFRAME ${current_airframe}" >&2
  exit 68
fi

mkdir -p "${analysis_dir}"
echo "LAYERED_TUNING_CAMPAIGN_BEGIN tag=${tag} max_iterations=${max_iterations}"
echo "LAYERED_TUNING_SOURCE ${current_airframe}"

for iteration in $(seq 1 "${max_iterations}"); do
  iteration_tag="${tag}_iter${iteration}"
  inner_prefix="inner_loop_identification_4kg_${iteration_tag}_inner"
  inner_decision="${analysis_dir}/${inner_prefix}_tuning_decision.json"
  inner_candidate="${analysis_dir}/${inner_prefix}_airframe_candidate_debug_4kg"
  combined_decision="${analysis_dir}/layered_tuning_decision_4kg_${iteration_tag}.json"
  combined_candidate="${analysis_dir}/layered_tuning_candidate_4kg_${iteration_tag}_debug_4kg"

  echo "LAYERED_TUNING_ITERATION_BEGIN iteration=${iteration} airframe=${current_airframe}"
  set +e
  RUN_LAYERED_ID_CONFIRM=1 \
  RUN_LAYERED_ID_TAKEOVER=1 \
  PROJECT_AIRFRAME_FILE="${current_airframe}" \
  bash "${layered_runner}" \
    "${iteration_tag}"
  layered_status=$?
  set -e

  decision_file=""
  candidate_file=""
  decision_scope=""
  if [[ -f "${combined_decision}" ]]; then
    decision_file="${combined_decision}"
    candidate_file="${combined_candidate}"
    decision_scope="combined"
  elif [[ -f "${inner_decision}" ]]; then
    decision_file="${inner_decision}"
    candidate_file="${inner_candidate}"
    decision_scope="inner"
  fi
  if [[ -z "${decision_file}" ]]; then
    echo "LAYERED_TUNING_STOP iteration=${iteration} reason=missing_decision status=${layered_status}" >&2
    exit 2
  fi

  decision="$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["decision"])' \
    "${decision_file}")"
  active_layer="$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("active_layer") or "NONE")' \
    "${decision_file}")"
  echo "LAYERED_TUNING_ITERATION_DECISION iteration=${iteration} decision=${decision} active_layer=${active_layer} status=${layered_status}"

  if [[ "${decision}" == "ALL_LAYERS_PASS_NO_PARAMETER_CHANGE" ]]; then
    if [[ ${layered_status} -ne 0 ]]; then
      echo "LAYERED_TUNING_STOP iteration=${iteration} reason=pass_decision_with_failed_runtime" >&2
      exit 3
    fi
    echo "LAYERED_TUNING_CAMPAIGN_PASS accepted_airframe=${current_airframe} decision=${decision_file}"
    exit 0
  fi

  if [[ "${decision}" == "BOUNDED_CANDIDATE_REQUIRES_RETEST" ]]; then
    # The layered runner exits 2 intentionally when an inner-loop candidate
    # must be retested before velocity identification.  A combined candidate
    # completes the runner with status 0.  Any other status means the flight
    # or analysis failed and its candidate must not be consumed.
    if ! { [[ "${decision_scope}" == "inner" && ${layered_status} -eq 2 ]] ||
           [[ "${decision_scope}" == "combined" && ${layered_status} -eq 0 ]]; }; then
      echo "LAYERED_TUNING_STOP iteration=${iteration} reason=candidate_from_failed_runtime scope=${decision_scope} status=${layered_status}" >&2
      exit 4
    fi
    if [[ ! -f "${candidate_file}" ]]; then
      echo "LAYERED_TUNING_STOP iteration=${iteration} reason=candidate_missing path=${candidate_file}" >&2
      exit 5
    fi
    if [[ "$(basename "${candidate_file}")" != *debug_4kg* ]]; then
      echo "LAYERED_TUNING_STOP iteration=${iteration} reason=non_debug_candidate" >&2
      exit 6
    fi
    echo "LAYERED_TUNING_RETEST iteration=${iteration} candidate=${candidate_file}"
    current_airframe="${candidate_file}"
    continue
  fi

  echo "LAYERED_TUNING_CAMPAIGN_HOLD iteration=${iteration} decision=${decision} active_layer=${active_layer} decision_file=${decision_file}"
  exit 2
done

echo "LAYERED_TUNING_MAX_ITERATIONS_REACHED max_iterations=${max_iterations} last_airframe=${current_airframe}" >&2
exit 3
