#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-}"
duration="${2:-3}"
case "${preset}" in
  retracted|work_a|work_b|flight_work_a|flight_work_b|demo_extended) ;;
  *) echo "usage: $0 {retracted|work_a|work_b|flight_work_a|flight_work_b|demo_extended} [duration_seconds]" >&2; exit 2 ;;
esac

source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"
exec ros2 run drone_arm_sim arm_preset_control \
  --preset "${preset}" --duration "${duration}" --wait --tolerance 0.06
