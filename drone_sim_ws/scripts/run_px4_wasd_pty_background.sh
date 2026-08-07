#!/usr/bin/env bash
set -euo pipefail

domain_id="${1:-50}"
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_file="/tmp/wasd_hybrid_test.out"
exit_file="/tmp/wasd_hybrid_test.exit"

set +u
source /opt/ros/jazzy/setup.bash
set -u
rm -f "${output_file}" "${exit_file}"
setsid -f env \
  ROS_DOMAIN_ID="${domain_id}" \
  GZ_PARTITION="${domain_id}" \
  IGN_PARTITION="${domain_id}" \
  bash -c \
  "cd '${workspace_dir}' && python3 scripts/test_px4_wasd_pty.py \
    >'${output_file}' 2>&1; printf '%s\n' \$? >'${exit_file}'"
