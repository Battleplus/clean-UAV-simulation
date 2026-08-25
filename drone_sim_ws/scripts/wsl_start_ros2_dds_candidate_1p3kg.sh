#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AIRFRAME_ID=4028
export PROJECT_AIRFRAME_FILE="${workspace_dir}/px4/airframes/4028_gz_my_drone_octorotor_candidate_1p3kg"
export ROBOT_FILE="${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
export SO101_KINEMATICS_URDF="${ROBOT_FILE}"
export CONFIG_FILE="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_candidate_1p3kg.json"
export MY_DRONE_FLIGHT_CONFIG="${CONFIG_FILE}"
export SO101_MOTION_REFERENCE="${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference_4kg.json"
export ARM_COUPLING_TARGET_MASS_KG=1.3
export BASE1_GRAVITY_TORQUE_LIMIT_NM=0.90
export BASE1_COMP_MAX_MOTOR_DELTA_N=1.25
# Candidate-only world-position force overlay.  The gains are mass-normalized
# pole placement for m=1.3 kg, wn=1.2 rad/s and damping ratio=0.9:
# Kp=m*wn^2 ~= 1.9 N/m, Kd=2*zeta*m*wn ~= 2.8 N*s/m.
# The 6D canted-rotor allocator supplies this horizontal force directly; this
# is a physical control input, not a Gazebo pose constraint.
export BASE1_POSITION_FEEDBACK_ENABLED="${BASE1_POSITION_FEEDBACK_ENABLED:-true}"
export BASE1_POSITION_GAIN_N_M="${BASE1_POSITION_GAIN_N_M:-1.9}"
export BASE1_VELOCITY_GAIN_N_S_M="${BASE1_VELOCITY_GAIN_N_S_M:-2.8}"
export BASE1_POSITION_HORIZONTAL_LIMIT_N="${BASE1_POSITION_HORIZONTAL_LIMIT_N:-0.20}"
export BASE1_POSITION_VERTICAL_LIMIT_N="${BASE1_POSITION_VERTICAL_LIMIT_N:-0.00}"
# While the arm session owns the direct 6D world-position loop, keep PX4's
# velocity request at zero instead of letting a second position loop command
# fore/aft tilt.  PX4 still closes velocity, attitude and rate control.
export PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY="${PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY:-false}"
# Candidate-only experiment: give the direct canted-rotor world-XY force loop
# sole ownership of horizontal position during a health-gated arm motion.
# Base1/4 kg launchers retain the controller's fail-closed default (false).
export ARM_DIRECT_XY_OWNERSHIP="${ARM_DIRECT_XY_OWNERSHIP:-true}"
export ARM_DIRECT_XY_EXTERNAL_GUARDIAN="${ARM_DIRECT_XY_EXTERNAL_GUARDIAN:-true}"
export ARM_DIRECT_XY_WATCHDOG_S="${ARM_DIRECT_XY_WATCHDOG_S:-0.04}"
export ARM_DIRECT_XY_HEALTH_HOLD_S="${ARM_DIRECT_XY_HEALTH_HOLD_S:-0.25}"
export ARM_DIRECT_XY_STABLE_HOLD_S="${ARM_DIRECT_XY_STABLE_HOLD_S:-0.50}"
# Match the arm executor's authoritative owner-entry contract.  Runtime
# freshness remains the independent 40 ms watchdog above.
export ARM_DIRECT_XY_ENTRY_TIMEOUT_S="${ARM_DIRECT_XY_ENTRY_TIMEOUT_S:-0.15}"
# The measured JointState second derivative is too noisy for a trustworthy
# rigid-body reaction estimate on this light candidate.  Keep the physically
# dominant pose/gravity compensation enabled, but make reaction feed-forward
# an explicit experiment until it is driven by the commanded q/qd/qdd path.
export BASE1_REACTION_FORCE_GAIN="${BASE1_REACTION_FORCE_GAIN:-0}"
export BASE1_REACTION_TORQUE_GAIN="${BASE1_REACTION_TORQUE_GAIN:-0}"

exec "${workspace_dir}/scripts/wsl_start_ros2_dds_debug_4kg.sh" "$@"
