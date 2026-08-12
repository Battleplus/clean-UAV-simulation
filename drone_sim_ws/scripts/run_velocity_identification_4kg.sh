#!/usr/bin/env bash
# Deterministic 10 Hz velocity/yaw identification for the disposable 4 kg model.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_dir="${workspace_dir}/analysis"
runtime_dir="/tmp/my_drone_ros2_dds"
tag="${1:-$(date +%Y%m%d_%H%M%S)}"
prefix="velocity_identification_4kg_${tag}"
raw_log="${analysis_dir}/${prefix}_raw.log"
report="${analysis_dir}/${prefix}.json"

if [[ "${RUN_VELOCITY_ID_CONFIRM:-0}" != "1" ]]; then
  echo "DRY_RUN: set RUN_VELOCITY_ID_CONFIRM=1 to stop the manual GUI and arm the 4 kg diagnostic" >&2
  exit 64
fi

interactive_processes=()
if pgrep -f '[g]z sim gui' >/dev/null 2>&1; then
  interactive_processes+=("gazebo_gui")
fi
if pgrep -f '[d]ds_wasd_control' >/dev/null 2>&1; then
  interactive_processes+=("dds_wasd_control")
fi
if (( ${#interactive_processes[@]} > 0 )) && \
   [[ "${RUN_VELOCITY_ID_TAKEOVER:-0}" != "1" ]]; then
  printf 'INTERACTIVE_RUNTIME_ACTIVE: %s\n' "${interactive_processes[*]}" >&2
  echo "REFUSED: velocity identification needs an exclusive backend." >&2
  echo "Close the GUI/WASD session, or explicitly set RUN_VELOCITY_ID_TAKEOVER=1." >&2
  exit 65
fi

mkdir -p "${analysis_dir}"
set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

echo "VELOCITY_ID_BACKEND_START tag=${tag}"
HEADLESS=true \
ENABLE_ARM_CONTROL=true \
CLEAN_STALE_RUNTIME=1 \
PX4_FRESH_WORKDIR=1 \
MODEL_SETTLE_S=8 \
ARM_TORQUE_FEEDFORWARD_ENABLED=false \
ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0 \
ARM_DISTURBANCE_OBSERVER_ENABLED=false \
bash "${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh"

set +e
python3 "${workspace_dir}/scripts/test_ros2_dds_velocity_wasd_pty.py" \
  --timeout 180 2>&1 | tee "${raw_log}"
flight_status=${PIPESTATUS[0]}
set -e

for name in agent gazebo px4 arm_init settle px4_stability; do
  if [[ -f "${runtime_dir}/${name}.log" ]]; then
    cp "${runtime_dir}/${name}.log" "${analysis_dir}/${prefix}_${name}.log"
  fi
done

set +e
python3 "${workspace_dir}/scripts/extract_velocity_identification.py" \
  "${raw_log}" --state-report-hz 10 --output "${report}"
extract_status=$?
set -e

echo "VELOCITY_ID_RESULT flight_status=${flight_status} extract_status=${extract_status}"
echo "VELOCITY_ID_RAW_LOG ${raw_log}"
echo "VELOCITY_ID_REPORT ${report}"
echo "NOTICE: headless diagnostic backend remains active; restore the standard GUI after review."

if [[ ${flight_status} -ne 0 || ${extract_status} -ne 0 ]]; then
  exit 1
fi
