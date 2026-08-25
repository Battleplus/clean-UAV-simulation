#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

echo "STAGE 1/2: 1.3 kg no-arm takeoff, hover, WASD and landing"
bash scripts/run_wasd_acceptance_1p3kg.sh

echo "STAGE 2/2: ten-direction arm extend, hold and full return"
bash scripts/run_directional_workspace_acceptance_1p3kg.sh

echo "CANDIDATE_1P3KG_FULL_ACCEPTANCE_PASS"
