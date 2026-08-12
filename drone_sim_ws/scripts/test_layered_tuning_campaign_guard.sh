#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
launcher="${workspace_dir}/scripts/run_layered_tuning_campaign_4kg.sh"
mock_runner="${workspace_dir}/scripts/test_fixtures/mock_layered_identification.sh"
source_airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"

set +e
dry_output="$(bash "${launcher}" guard_dry 2>&1)"
dry_status=$?
set -e
[[ ${dry_status} -eq 64 ]]
grep -q 'DRY_RUN' <<<"${dry_output}"

set +e
range_output="$(RUN_LAYERED_TUNING_CONFIRM=1 LAYERED_TUNING_MAX_ITERATIONS=7 \
  bash "${launcher}" guard_range 2>&1)"
range_status=$?
set -e
[[ ${range_status} -eq 66 ]]
grep -q 'INVALID_MAX_ITERATIONS' <<<"${range_output}"

if pgrep -f '[g]z sim gui|[d]ds_wasd_control' >/dev/null 2>&1; then
  set +e
  active_output="$(RUN_LAYERED_TUNING_CONFIRM=1 bash "${launcher}" guard_active 2>&1)"
  active_status=$?
  set -e
  [[ ${active_status} -eq 65 ]]
  grep -q 'INTERACTIVE_RUNTIME_ACTIVE' <<<"${active_output}"
  grep -q 'needs an exclusive backend' <<<"${active_output}"
  echo "LAYERED_TUNING_CAMPAIGN_GUARD_PASS active_runtime_refused"
else
  echo "LAYERED_TUNING_CAMPAIGN_GUARD_PASS no_interactive_runtime"
fi

run_mock_campaign() {
  local sequence="$1"
  local expected_status="$2"
  local temp_dir
  temp_dir="$(mktemp -d /tmp/my_drone_layered_campaign_test.XXXXXX)"
  set +e
  output="$(RUN_LAYERED_TUNING_CONFIRM=1 \
    RUN_LAYERED_TUNING_TAKEOVER=1 \
    LAYERED_TUNING_MAX_ITERATIONS=3 \
    LAYERED_TUNING_ANALYSIS_DIR="${temp_dir}" \
    LAYERED_ID_RUNNER="${mock_runner}" \
    MOCK_LAYERED_SEQUENCE="${sequence}" \
    PROJECT_AIRFRAME_FILE="${source_airframe}" \
    bash "${launcher}" "mock_${sequence}" 2>&1)"
  status=$?
  set -e
  if [[ ${status} -ne ${expected_status} ]]; then
    printf '%s\n' "${output}" >&2
    echo "MOCK_CAMPAIGN_STATUS_FAIL sequence=${sequence} expected=${expected_status} actual=${status}" >&2
    exit 1
  fi
  case "${temp_dir}" in
    /tmp/my_drone_layered_campaign_test.*) rm -rf -- "${temp_dir}" ;;
    *) echo "REFUSED_UNSAFE_TEST_CLEANUP ${temp_dir}" >&2; exit 1 ;;
  esac
  printf '%s\n' "${output}"
}

pass_output="$(run_mock_campaign inner_candidate_then_pass 0)"
grep -q 'LAYERED_TUNING_RETEST iteration=1' <<<"${pass_output}"
grep -q 'LAYERED_TUNING_CAMPAIGN_PASS' <<<"${pass_output}"

failed_output="$(run_mock_campaign failed_candidate 4)"
grep -q 'candidate_from_failed_runtime' <<<"${failed_output}"

hold_output="$(run_mock_campaign hold 2)"
grep -q 'LAYERED_TUNING_CAMPAIGN_HOLD' <<<"${hold_output}"

echo "LAYERED_TUNING_CAMPAIGN_BRANCH_PASS"
