#!/usr/bin/env bash
# Start the 100 Hz read-only estimator without modifying Base 1 launch defaults.

set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="/tmp/my_drone_ros2_dds"
cpu_role_runner="${workspace_dir}/scripts/run_with_cpu_role.sh"
motion_reference="${SO101_MOTION_REFERENCE:-${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference.json}"
kinematics_urdf="${SO101_KINEMATICS_URDF:-${ROBOT_FILE:-${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf}}"
target_mass_kg="${ARM_COUPLING_TARGET_MASS_KG:-4.0}"
mkdir -p "${runtime_dir}"

estimator_pid_file="${runtime_dir}/base1_estimator.pid"
estimator_child_pattern='/[a]rm_coupling_monitor .*__node:=base1_arm_coupling_estimator_100hz'

# `ros2 run` is a Python wrapper which starts the installed node as a child.
# Because this script uses `setsid`, the wrapper PID is also the process-group
# ID.  Killing only the PID or only the child leaves the other half behind and
# was the source of duplicate estimator node names on a later overlay start.
stop_previous_estimator() {
  local prior="" command_line="" pgid=""
  if [[ -f "${estimator_pid_file}" ]]; then
    prior="$(tr -cd '0-9' <"${estimator_pid_file}")"
    if [[ -n "${prior}" ]] && kill -0 "${prior}" 2>/dev/null; then
      command_line="$(ps -o args= -p "${prior}" 2>/dev/null || true)"
      if [[ "${command_line}" == *"ros2 run drone_arm_sim arm_coupling_monitor"* ]] && \
          [[ "${command_line}" == *"base1_arm_coupling_estimator_100hz"* ]]; then
        pgid="$(ps -o pgid= -p "${prior}" 2>/dev/null | tr -d ' ' || true)"
        if [[ -n "${pgid}" && "${pgid}" == "${prior}" ]]; then
          kill -TERM -- "-${pgid}" 2>/dev/null || true
        else
          kill -TERM "${prior}" 2>/dev/null || true
        fi
      fi
    fi
    rm -f "${estimator_pid_file}"
  fi

  # Also cover a lost/stale PID file.  This expression matches only the
  # installed estimator child carrying the dedicated ROS node rename, not the
  # frozen 3 Hz Base1 monitor and not this shell.
  pkill -TERM -f "${estimator_child_pattern}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    pgrep -f "${estimator_child_pattern}" >/dev/null 2>&1 || return 0
    sleep 0.1
  done
  pkill -KILL -f "${estimator_child_pattern}" 2>/dev/null || true
}

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

# ROS graph discovery can briefly retain or lose a topic while the Base 1
# launcher finishes activating ros2_control.  Use one DDS participant to
# require multiple real samples.  Repeated one-shot topic reader processes
# each redo discovery and can hold the complete startup chain for minutes on
# WSL even while the publisher is healthy.
startup_sample_timeout_s="${BASE1_STARTUP_SAMPLE_TIMEOUT_S:-8}"
if ! python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
    --joint-samples 3 --timeout "${startup_sample_timeout_s}"; then
  echo "REFUSED: /joint_states did not remain continuously available; start the Base 1 arm runtime first" >&2
  exit 2
fi

stop_previous_estimator

setsid "${cpu_role_runner}" bulk \
  ros2 run drone_arm_sim arm_coupling_monitor \
  --urdf "${kinematics_urdf}" \
  --motion-reference "${motion_reference}" \
  --rate-hz 100 \
  --target-mass-kg "${target_mass_kg}" \
  --payload-mass-kg 0.0 \
  --feedforward-limit-m-s2 0.0 \
  --ros-args \
  -r __node:=base1_arm_coupling_estimator_100hz \
  -r /my_drone/arm_reaction_wrench_body:=/my_drone/base1_estimator/reaction_wrench_body \
  -r /my_drone/arm_gravity_shift_wrench_body:=/my_drone/base1_estimator/gravity_shift_wrench_body \
  -r /my_drone/arm_feedforward_acceleration_ned:=/my_drone/base1_estimator/candidate_acceleration_ned \
  -r /my_drone/arm_coupling_state:=/my_drone/base1_estimator/coupling_state \
  >"${runtime_dir}/base1_estimator.log" 2>&1 &
pid=$!
echo "${pid}" >"${estimator_pid_file}"

# One DDS participant waits for the real data stream.  Avoid repeated
# `ros2 topic list` processes: on WSL each one can redo discovery and make a
# healthy startup look hung.
if python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
    --joint-samples 1 --coupling-samples 3 \
    --timeout "${startup_sample_timeout_s}"; then
  mapfile -t estimator_children < <(pgrep -f "${estimator_child_pattern}" || true)
  if [[ "${#estimator_children[@]}" -eq 1 ]]; then
    echo "BASE1_READONLY_ESTIMATOR_READY pid=${pid} child_pid=${estimator_children[0]} rate_hz=100 mass_kg=${target_mass_kg}"
    exit 0
  fi
  echo "REFUSED: expected one 100 Hz estimator child, found ${#estimator_children[@]}" >&2
fi

echo "Base 1 estimator overlay topic did not appear" >&2
if kill -0 "${pid}" 2>/dev/null; then
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
else
  cat "${runtime_dir}/base1_estimator.log" >&2 || true
fi
rm -f "${estimator_pid_file}"
exit 1
