#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source install/setup.bash

log_file="${1:-${ARM_FLIGHT_LOG:-analysis/cartesian_demo_flight_twice_4kg.log}}"
exit_file="${log_file%.log}.exit"
rm -f "$exit_file"

ARM_FLIGHT_PROFILE=cartesian_demo_twice_4kg \
  python3 scripts/test_ros2_dds_arm_flight_pty.py --timeout 320 \
  2>&1 | tee "$log_file"
result=${PIPESTATUS[0]}
printf '%s\n' "$result" > "$exit_file"
exit "$result"
