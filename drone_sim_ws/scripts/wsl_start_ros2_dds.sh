#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ENABLE_ARM_CONTROL="${ENABLE_ARM_CONTROL:-true}"
exec bash "${script_dir}/wsl_start_ros2_dds_noarm.sh"
