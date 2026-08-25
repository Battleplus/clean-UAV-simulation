#!/usr/bin/env bash
# Isolate the reported 1.3 kg failure: nose-forward extend, hold, and exact
# return to the folded pose.  The full ten-direction runner remains unchanged
# unless ARM_DIRECTIONAL_ONLY is explicitly supplied.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

export ARM_DIRECTIONAL_ONLY=front
export ARM_DIRECTIONAL_PREFLIGHT_SAMPLES=81

exec bash scripts/run_directional_workspace_acceptance_1p3kg.sh \
  "${1:-analysis/base1/front_retract_flight_acceptance_1p3kg.log}"
