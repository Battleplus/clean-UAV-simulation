#!/usr/bin/env bash
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}" || exit 1
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source install/setup.bash

log_file="${1:-analysis/nose_forward_straight_flight_4kg.log}"
exit_file="${log_file%.log}.exit"
rm -f "${exit_file}"

# First-stage unified acceptance requested by the project goal.
ARM_FLIGHT_PROFILE=nose_forward_straight_4kg \
ARM_NOSE_FORWARD_EXTENSION_DURATION_S="${ARM_NOSE_FORWARD_EXTENSION_DURATION_S:-90}" \
ARM_NOSE_FORWARD_RETURN_DURATION_S="${ARM_NOSE_FORWARD_RETURN_DURATION_S:-120}" \
ARM_FLIGHT_ACCEPT_HORIZONTAL_M=0.05 \
ARM_FLIGHT_ACCEPT_ALTITUDE_M=0.05 \
ARM_FLIGHT_ACCEPT_TILT_DEG=1.0 \
  python3 scripts/test_ros2_dds_arm_flight_pty.py --timeout 330 \
  2>&1 | tee "${log_file}"
result=${PIPESTATUS[0]}
printf '%s\n' "${result}" > "${exit_file}"
exit "${result}"
