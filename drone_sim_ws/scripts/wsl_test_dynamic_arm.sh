#!/usr/bin/env bash
set -e

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GZ_PARTITION=my_drone_dynamic_arm
source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
source install/setup.bash
exec >"${workspace_dir}/analysis/cad_direct/dynamic_arm_launch.log" 2>&1
exec ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true \
  enable_controller:=false \
  spawn_z:=0.45
