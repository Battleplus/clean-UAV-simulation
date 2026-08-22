#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe_id="${AIRFRAME_ID:-4027}"
airframe="${PROJECT_AIRFRAME_FILE:-${workspace_dir}/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg}"
airframe_name="$(basename "${airframe}")"
if [[ "${airframe_name}" != "${airframe_id}_"* ]]; then
  echo "Airframe file ${airframe_name} does not match AIRFRAME_ID=${airframe_id}" >&2
  exit 1
fi
px4_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/${airframe_name}"

if [[ ! -f "${airframe}" ]]; then
  echo "4 kg debug profile is missing; run scripts/build_4kg_debug_profile.py first" >&2
  exit 1
fi
cp "${airframe}" "${px4_airframe}"
chmod +x "${px4_airframe}"

export AIRFRAME_ID="${airframe_id}"
export PROJECT_AIRFRAME_FILE="${PROJECT_AIRFRAME_FILE:-${airframe}}"
export ROBOT_FILE="${ROBOT_FILE:-${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf}"
export CONFIG_FILE="${CONFIG_FILE:-${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json}"
export MY_DRONE_WORLD="${workspace_dir}/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf"
# Keep 4.0 kg as the validated debug default, but allow payload-derived
# profiles to pass their analyzed total mass without editing this launcher.
export ARM_COUPLING_TARGET_MASS_KG="${ARM_COUPLING_TARGET_MASS_KG:-4.0}"
export BATTERY_DYNAMICS_ENABLED=false
# This node is also the raw->PX4 Gazebo topic relay.  Keep it running while
# setting every delay below to zero; disabling the node removes all IMU/GNSS
# input rather than merely removing latency.
export ENABLE_SENSOR_DELAY=true
export IMU_DELAY_MS=0
export MAG_DELAY_MS=0
export BARO_DELAY_MS=0
export NAVSAT_DELAY_MS=0
# Experimental yaw-authority value for the 4 kg calibration profile only.
# Real propeller reaction torque is still unresolved and must replace this
# value before the formal 7.735 kg model can be called physically frozen.
export REACTION_MOMENT_RATIO_M="${REACTION_MOMENT_RATIO_M:-0.005}"
# The PX4 profile retains 0.25 m/s up/down safety limits.  A gentler manual
# command avoids the measured descent overshoot seen with a 0.25 m/s key
# command while preserving responsive R/F control.
export PX4_WASD_VERTICAL_SPEED_M_S="${PX4_WASD_VERTICAL_SPEED_M_S:-0.15}"
# Base 1 H-hover uses Gazebo odometry as an ideal motion-capture outer loop.
# WASD remains the original latched velocity controller; PX4 still closes the
# velocity, attitude and rate loops and the arm disturbance remains physical.
export PX4_TRUTH_HOLD_ENABLED="${PX4_TRUTH_HOLD_ENABLED:-true}"
export PX4_TOUCHDOWN_DISARM_ENABLED=true
export PX4_TOUCHDOWN_DISARM_HEIGHT_M="${PX4_TOUCHDOWN_DISARM_HEIGHT_M:-0.10}"
export PX4_TOUCHDOWN_DISARM_HOLD_S=0.5
export SPAWN_Z="${SPAWN_Z:-0.289}"

"${workspace_dir}/scripts/wsl_start_ros2_dds_noarm.sh"

# The accepted Base 1 arm configuration is enabled only for the 4 kg profile
# and only when the movable arm is present. PX4 keeps all flight loops. The
# overlay contributes the static COM gravity torque and the six-dimensional
# rigid-body momentum reaction wrench. Joint damping/friction is deliberately
# excluded in coupled_dynamics because Gazebo already transmits that internal
# actuator-load pair through the articulated body.
if [[ "${ENABLE_ARM_CONTROL:-false}" == "true" \
  && "${BASE1_AUTO_ARM_COMPENSATION:-true}" == "true" ]]; then
  BASE1_COMPENSATION_ENABLED=true \
  BASE1_REACTION_FORCE_GAIN=1 \
  BASE1_REACTION_TORQUE_GAIN=1 \
  BASE1_GRAVITY_TORQUE_GAIN=1 \
  BASE1_GRAVITY_TORQUE_LIMIT_NM=0.65 \
  BASE1_COMP_MAX_MOTOR_DELTA_N=0.70 \
  BASE1_POSITION_FEEDBACK_ENABLED=false \
    "${workspace_dir}/scripts/activate_base1_wrench_reallocator_overlay.sh"
  echo "BASE1_ARM_COMPENSATION_READY gravity=true dynamic_wrench_6d=true"
fi
