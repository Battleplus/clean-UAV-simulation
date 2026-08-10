#!/usr/bin/env bash
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source install/setup.bash

distance="${ARM_CARTESIAN_DISTANCE_M:-0.05}"
distance_tag="${distance//./p}"
run_tag="${FORMAL_RUN_TAG:-baseline}"
log="analysis/cartesian_formal_7p735_${distance_tag}m_${run_tag}.log"
status="analysis/cartesian_formal_7p735_${distance_tag}m_${run_tag}.status"
rm -f "$status"

ARM_FLIGHT_PROFILE=cartesian_formal_7p735 \
ARM_CARTESIAN_DISTANCE_M="$distance" \
python3 scripts/test_ros2_dds_arm_flight_pty.py --timeout "${FORMAL_TIMEOUT_S:-300}" \
  >"$log" 2>&1
result=$?
printf '%s\n' "$result" >"$status"
exit "$result"
