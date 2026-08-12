#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
launcher="${workspace_dir}/scripts/run_inner_loop_identification_4kg.sh"

set +e
output="$(RUN_INNER_LOOP_ID_CONFIRM=1 bash "${launcher}" guard_test 2>&1)"
status=$?
set -e

if pgrep -f '[g]z sim gui|[d]ds_wasd_control' >/dev/null 2>&1; then
  if [[ ${status} -ne 65 ]]; then
    echo "INNER_LOOP_RUNTIME_GUARD_FAIL expected=65 actual=${status}" >&2
    printf '%s\n' "${output}" >&2
    exit 1
  fi
  grep -q 'INTERACTIVE_RUNTIME_ACTIVE' <<<"${output}"
  grep -q 'REFUSED: V3 identification needs an exclusive backend' <<<"${output}"
  echo "INNER_LOOP_RUNTIME_GUARD_PASS active_runtime_refused"
else
  echo "INNER_LOOP_RUNTIME_GUARD_SKIP no interactive runtime"
fi
