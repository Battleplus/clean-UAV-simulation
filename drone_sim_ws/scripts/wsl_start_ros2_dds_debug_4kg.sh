#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe="${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
px4_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/4027_gz_my_drone_octorotor_debug_4kg"

if [[ ! -f "${airframe}" ]]; then
  echo "4 kg debug profile is missing; run scripts/build_4kg_debug_profile.py first" >&2
  exit 1
fi
cp "${airframe}" "${px4_airframe}"
chmod +x "${px4_airframe}"

export AIRFRAME_ID=4027
export PROJECT_AIRFRAME_FILE="${workspace_dir}/px4/airframes/.4027_already_installed"
export ROBOT_FILE="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
export MY_DRONE_WORLD="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
export ARM_COUPLING_TARGET_MASS_KG=4.0
export BATTERY_DYNAMICS_ENABLED=false
# This node is also the raw->PX4 Gazebo topic relay.  Keep it running while
# setting every delay below to zero; disabling the node removes all IMU/GNSS
# input rather than merely removing latency.
export ENABLE_SENSOR_DELAY=true
export IMU_DELAY_MS=0
export MAG_DELAY_MS=0
export BARO_DELAY_MS=0
export NAVSAT_DELAY_MS=0
export REACTION_MOMENT_RATIO_M=0.001
# The PX4 profile retains 0.25 m/s up/down safety limits.  A gentler manual
# command avoids the measured descent overshoot seen with a 0.25 m/s key
# command while preserving responsive R/F control.
export PX4_WASD_VERTICAL_SPEED_M_S="${PX4_WASD_VERTICAL_SPEED_M_S:-0.15}"
export PX4_TOUCHDOWN_DISARM_ENABLED=true
export PX4_TOUCHDOWN_DISARM_HEIGHT_M=0.05
export PX4_TOUCHDOWN_DISARM_HOLD_S=0.5
export SPAWN_Z="${SPAWN_Z:-0.183}"

exec "${workspace_dir}/scripts/wsl_start_ros2_dds_noarm.sh"
