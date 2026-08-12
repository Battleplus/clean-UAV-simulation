#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
launcher="${workspace_dir}/scripts/run_deterministic_arm_dob_ab.sh"

set +e
output="$(ARM_DOB_AB_GAIN=0.75 RUN_DOB_AB_CONFIRM=1 bash "${launcher}" gain_guard 2>&1)"
status=$?
set -e

if [[ ${status} -ne 66 ]]; then
  echo "DOB_GAIN_GUARD_FAIL expected=66 actual=${status}" >&2
  printf '%s\n' "${output}" >&2
  exit 1
fi
grep -q 'ARM_DOB_AB_GAIN must be finite and in (0, 0.5]' <<<"${output}"
echo "DOB_GAIN_GUARD_PASS invalid_high_gain_refused"
