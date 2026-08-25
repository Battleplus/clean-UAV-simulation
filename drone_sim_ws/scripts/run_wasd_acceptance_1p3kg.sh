#!/usr/bin/env bash
set -euo pipefail
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source install/setup.bash
set -u

colcon build --symlink-install --packages-select drone_arm_sim px4_ros2_control \
  2>&1 | tee analysis/base1/wasd_1p3kg_colcon_build.log
set +u
source install/setup.bash
set -u

HEADLESS="${WASD_ACCEPTANCE_HEADLESS:-true}" \
ENABLE_ARM_CONTROL=false \
ARM_DIRECT_XY_OWNERSHIP=false \
ARM_DIRECT_XY_EXTERNAL_GUARDIAN=false \
CLEAN_STALE_RUNTIME=1 \
PX4_FRESH_WORKDIR=1 \
  bash scripts/wsl_start_ros2_dds_candidate_1p3kg.sh \
  2>&1 | tee analysis/base1/wasd_1p3kg_backend.log

set +e
PX4_TRUTH_HOLD_ENABLED=true \
PX4_TOUCHDOWN_DISARM_ENABLED=true \
PX4_TOUCHDOWN_DISARM_HEIGHT_M="${PX4_TOUCHDOWN_DISARM_HEIGHT_M:-0.10}" \
PX4_TOUCHDOWN_CONTACT_ALLOWANCE_M="${PX4_TOUCHDOWN_CONTACT_ALLOWANCE_M:-0.02}" \
PX4_TOUCHDOWN_DISARM_HOLD_S="${PX4_TOUCHDOWN_DISARM_HOLD_S:-0.5}" \
ARM_DIRECT_XY_OWNERSHIP=false \
ARM_DIRECT_XY_EXTERNAL_GUARDIAN=false \
  python3 scripts/test_ros2_dds_wasd_pty.py --timeout "${WASD_ACCEPTANCE_TIMEOUT_S:-110}" \
  2>&1 | tee analysis/base1/wasd_flight_acceptance_1p3kg.log
result=${PIPESTATUS[0]}
set -e
printf '%s\n' "${result}" > analysis/base1/wasd_flight_acceptance_1p3kg.exit
exit "${result}"
