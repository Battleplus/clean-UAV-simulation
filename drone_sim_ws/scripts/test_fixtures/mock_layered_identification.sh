#!/usr/bin/env bash
# Deterministic file-producing stand-in for campaign control-flow tests only.

set -euo pipefail

tag="$1"
analysis_dir="${LAYERED_TUNING_ANALYSIS_DIR:?}"
sequence="${MOCK_LAYERED_SEQUENCE:?}"
inner_prefix="inner_loop_identification_4kg_${tag}_inner"
inner_decision="${analysis_dir}/${inner_prefix}_tuning_decision.json"
inner_candidate="${analysis_dir}/${inner_prefix}_airframe_candidate_debug_4kg"
combined_decision="${analysis_dir}/layered_tuning_decision_4kg_${tag}.json"

mkdir -p "${analysis_dir}"

write_decision() {
  local path="$1"
  local decision="$2"
  local layer="$3"
  printf '{"decision":"%s","active_layer":%s}\n' \
    "${decision}" "${layer}" >"${path}"
}

case "${sequence}:${tag}" in
  inner_candidate_then_pass:*_iter1)
    write_decision "${inner_decision}" "BOUNDED_CANDIDATE_REQUIRES_RETEST" '"body_rate"'
    printf 'mock candidate\n' >"${inner_candidate}"
    exit 2
    ;;
  inner_candidate_then_pass:*_iter2)
    write_decision "${combined_decision}" "ALL_LAYERS_PASS_NO_PARAMETER_CHANGE" null
    exit 0
    ;;
  failed_candidate:*)
    write_decision "${inner_decision}" "BOUNDED_CANDIDATE_REQUIRES_RETEST" '"body_rate"'
    printf 'invalid mock candidate\n' >"${inner_candidate}"
    exit 1
    ;;
  hold:*)
    write_decision "${inner_decision}" "HOLD_INCOMPLETE_IDENTIFICATION" '"body_rate"'
    exit 2
    ;;
  *)
    echo "unexpected mock sequence/tag ${sequence}:${tag}" >&2
    exit 99
    ;;
esac
