#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

export ARM_WORKSPACE_ENVELOPE="analysis/base1/arm_workspace_envelope_1p3kg.json"
export ARM_DIRECTIONAL_PLAN="analysis/base1/directional_workspace_flight_plan_1p3kg.json"
export ARM_CANDIDATE_URDF="src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
export ARM_CANDIDATE_MOTION_REFERENCE="src/drone_arm_sim/config/so101_motion_reference_4kg.json"
export ARM_CANDIDATE_FLIGHT_CONFIG="src/drone_arm_sim/config/my_drone_v3_cad_candidate_1p3kg.json"
export ARM_CANDIDATE_MASS_KG=1.3
export ARM_CANDIDATE_GRAVITY_TORQUE_LIMIT_NM=0.90
export ARM_CANDIDATE_MAXIMUM_MOTOR_DELTA_N=1.25
export ARM_DIRECTIONAL_SHOULDER_PAN_SPEED_CAP_RAD_S=0.05
# The 4 kg Base1 candidate is stable with the standard 0.80 position-to-
# velocity outer-loop gain.  On the 1.3 kg candidate, the same command creates
# a measured XY limit cycle before the arm even starts (up to about 0.21 m/s),
# so the strict direct-owner entry gate can never be reached.  Keep the PX4
# velocity/attitude/rate stack unchanged and reduce only this candidate's
# truth-hold outer-loop command.  These values do not relax any acceptance or
# watchdog threshold; they must earn the existing 0.05 m / 0.08 m/s / 1 deg
# continuous entry window in flight.
export PX4_TRUTH_HOLD_XY_P="${PX4_TRUTH_HOLD_XY_P:-0.25}"
export PX4_TRUTH_HOLD_XY_D="${PX4_TRUTH_HOLD_XY_D:-0.0}"
export PX4_TRUTH_HOLD_XY_MAX_M_S="${PX4_TRUTH_HOLD_XY_MAX_M_S:-0.04}"
# The 41-point diagnostic sampling missed an intermediate upper-arm/wrist
# collision on a vertical candidate.  Keep the 1.3 kg candidate on the dense
# continuous-path audit that selected the currently accepted safe endpoint.
export ARM_DIRECTIONAL_PREFLIGHT_SAMPLES="${ARM_DIRECTIONAL_PREFLIGHT_SAMPLES:-81}"
export ARM_CANDIDATE_BACKEND_LAUNCHER="scripts/wsl_start_ros2_dds_candidate_1p3kg.sh"
export MY_DRONE_FLIGHT_CONFIG="${ARM_CANDIDATE_FLIGHT_CONFIG}"
export SO101_KINEMATICS_URDF="${ARM_CANDIDATE_URDF}"
export SO101_MOTION_REFERENCE="${ARM_CANDIDATE_MOTION_REFERENCE}"
export ARM_DIRECT_XY_OWNERSHIP="${ARM_DIRECT_XY_OWNERSHIP:-true}"
export ARM_DIRECT_XY_EXTERNAL_GUARDIAN="${ARM_DIRECT_XY_EXTERNAL_GUARDIAN:-true}"
# Reserve separate inherited CPU affinities for the guardian, PX4 keyboard
# controller, arm command process and reallocator.  Gazebo/PX4 and evidence
# recording use the remaining bulk pool.  This is non-root scheduling only;
# every watchdog and acceptance threshold remains unchanged.
export MY_DRONE_CPU_ISOLATION_ENABLED="${MY_DRONE_CPU_ISOLATION_ENABLED:-true}"

exec bash scripts/run_directional_workspace_acceptance_4kg.sh \
  "${1:-analysis/base1/directional_workspace_flight_acceptance_1p3kg.log}"
