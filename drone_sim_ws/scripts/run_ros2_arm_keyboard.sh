#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
export SO101_KINEMATICS_URDF="${SO101_KINEMATICS_URDF:-${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf}"
export SO101_MOTION_REFERENCE="${SO101_MOTION_REFERENCE:-${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference_4kg.json}"
export MY_DRONE_FLIGHT_CONFIG="${MY_DRONE_FLIGHT_CONFIG:-${CONFIG_FILE:-${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json}}"

# Preserve the reason for an interactive command failure even when the
# Windows terminal is closed later.  This does not redirect stdin, so the
# single-key controller remains interactive.
runtime_dir="${ARM_KEYBOARD_RUNTIME_DIR:-/tmp/my_drone_ros2_dds}"
mkdir -p "${runtime_dir}"
exec > >(tee -a "${runtime_dir}/arm_keyboard.log") 2>&1

# Keep flight and arm input windows visually distinct, and show the model that
# this terminal will actually command.  A hard-coded "4kg" title made the
# 1.3 kg candidate window look like the obsolete controller.
profile_label="4kg Base1"
if [[ "$(basename "${MY_DRONE_FLIGHT_CONFIG}")" == *"1p3kg"* ]]; then
  profile_label="1.3kg candidate"
fi
printf '\033]0;my_drone %s 联合调试 - SO101 机械臂\007' "${profile_label}"

run_preset() {
  local preset="$1"
  local duration="$2"
  local tolerance="${3:-0.08}"
  local arming_state="${4:-}"
  local direct_xy_ownership=false
  local direct_xy_external_guardian=false
  echo "Sending ${preset} (${duration}s)..."
  if [[ -z "${arming_state}" ]]; then
    if ! arming_state="$(get_vehicle_arming_state)"; then
      echo "ARM_COMMAND_REFUSED reason=vehicle_arming_state_unknown"
      return 2
    fi
  fi
  local preflight_args=()
  if [[ "${arming_state}" == "armed" ]]; then
    preflight_args+=(--flight-preflight)
    direct_xy_ownership="${ARM_DIRECT_XY_OWNERSHIP:-false}"
    direct_xy_external_guardian="${ARM_DIRECT_XY_EXTERNAL_GUARDIAN:-false}"
  fi
  # Ground motion must not request airborne XY ownership.  With the candidate
  # launcher exporting ARM_DIRECT_XY_OWNERSHIP=true globally, the old path
  # waited for a flight intent that cannot exist while disarmed and timed out.
  ARM_DIRECT_XY_OWNERSHIP="${direct_xy_ownership}" \
  ARM_DIRECT_XY_EXTERNAL_GUARDIAN="${direct_xy_external_guardian}" \
    ros2 run drone_arm_sim arm_preset_control \
    --preset "${preset}" --duration "${duration}" --wait --tolerance "${tolerance}" \
    "${preflight_args[@]}"
}

run_ground_preset() {
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "GROUND_ONLY_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    echo "GROUND_ONLY_REFUSED: full diagnostic poses 4/5 are disabled in flight"
    discard_buffered_keys
    return 1
  fi
  run_preset "$1" "$2" 0.08 "${arming_state}"
  discard_buffered_keys
}

get_vehicle_arming_state() {
  # One rclpy participant waits for a real PX4 sample.  A DDS discovery
  # timeout is an unknown state, never evidence that the aircraft is
  # disarmed.  This prevents one keypress from taking the ground branch and a
  # later check of the same command from taking the airborne branch.
  local output=""
  if ! output="$(python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
      --vehicle-status-samples 1 --print-arming-state \
      --timeout "${ARM_VEHICLE_STATUS_TIMEOUT_S:-12}")"; then
    echo "unknown"
    return 2
  fi
  if grep -q 'VEHICLE_ARMING_STATE state=2' <<<"${output}"; then
    echo "armed"
    return 0
  fi
  if grep -q 'VEHICLE_ARMING_STATE state=1' <<<"${output}"; then
    echo "disarmed"
    return 0
  fi
  # Boot/standby or any future enum value is not proof of a safely disarmed
  # flight state.  Fail closed instead of silently choosing the ground path.
  echo "unknown"
  return 2
}

request_hover_and_wait_stable() {
  echo "AIRBORNE_STABILITY_GATE: requesting H-equivalent hover and waiting for 5 stable seconds"
  ros2 topic pub --once /my_drone/hover_request std_msgs/msg/Bool '{data: true}' >/dev/null
  python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
    --horizontal 0.10 --vertical 0.08 --hold 5 --timeout 90
}

prepare_keyboard_motion() {
  local arming_state="${1:-}"
  if [[ -z "${arming_state}" ]]; then
    if ! arming_state="$(get_vehicle_arming_state)"; then
      echo "AIRBORNE_JOG_REFUSED: PX4 arming state is unavailable"
      return 1
    fi
  fi
  if [[ "${arming_state}" != "armed" ]]; then
    return 0
  fi
  echo "AIRBORNE_JOG_GATE: requesting zero velocity and checking stability"
  ros2 topic pub --once /my_drone/hover_request std_msgs/msg/Bool '{data: true}' >/dev/null
  python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
    --horizontal 0.10 --vertical 0.08 --hold 1.5 --timeout 20
}

run_flight_preset() {
  local preset="$1"
  local duration="$2"
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "PRESET_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    echo "AIRBORNE_PRESET_GATE: requesting zero velocity before ${preset}"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_PRESET_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  run_preset "${preset}" "${duration}" 0.08 "${arming_state}"
  discard_buffered_keys
}

run_jog() {
  local joint="$1"
  local delta="$2"
  local duration=1.5
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "JOG_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    duration=2.5
    if ! prepare_keyboard_motion "${arming_state}"; then
      echo "AIRBORNE_JOG_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  echo "JOG ${joint} ${delta}rad (${duration}s)"
  ros2 run drone_arm_sim arm_joint_jog \
    --joint "${joint}" --delta "${delta}" --duration "${duration}"
  discard_buffered_keys
}

run_gripper_preset() {
  local preset="$1"
  local duration=3
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "GRIPPER_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    duration=6
    if ! prepare_keyboard_motion "${arming_state}"; then
      echo "AIRBORNE_GRIPPER_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  run_preset "${preset}" "${duration}" 0.08 "${arming_state}"
  discard_buffered_keys
}

discard_buffered_keys() {
  local ignored
  # Keys pressed while a trajectory is running remain queued in the terminal.
  # Discard them so repeatedly tapping 6 cannot silently launch another full
  # cycle as soon as the first one returns.
  while IFS= read -rsn1 -t 0.01 ignored; do :; done
}

require_fresh_arm_runtime() {
  # A surviving PX4/DDS process is not sufficient: Gazebo can already have
  # stopped, leaving cached vehicle state while /clock and /joint_states are
  # frozen.  Refuse motion before entering a long hover gate in that state.
  if ! python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
      --joint-samples 3 --timeout 5; then
    echo "ARM_RUNTIME_UNAVAILABLE reason=joint_states_stale_or_gazebo_stopped"
    return 1
  fi
  local controller_info=""
  if ! controller_info="$(timeout 5 ros2 topic info /arm_controller/joint_trajectory 2>/dev/null)" \
      || ! grep -Eq 'Subscription count: [1-9][0-9]*' <<<"${controller_info}"; then
    echo "ARM_RUNTIME_UNAVAILABLE reason=arm_controller_missing"
    return 1
  fi
  return 0
}

run_visible_demo() {
  local duration=20
  local return_duration=24
  local hold=3
  local arming_state=""
  if ! require_fresh_arm_runtime; then
    echo "VISIBLE_DEMO_REFUSED: Gazebo or the arm controller is not ready; restart the complete candidate runtime"
    discard_buffered_keys
    return 1
  fi
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "VISIBLE_DEMO_REFUSED: PX4 arming state is unavailable; no trajectory was sent"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    duration="${ARM_KEY6_AIRBORNE_DURATION_S:-90}"
    # Retraction reduces the articulated inertia while the gravity moment is
    # changing in the opposite direction.  The measured Base1 run showed the
    # return pitch-rate peak was more than twice the extension peak, so give
    # the return its own lower acceleration/jerk budget instead of forcing a
    # nominally symmetric time law onto an asymmetric aircraft.
    return_duration="${ARM_KEY6_AIRBORNE_RETURN_DURATION_S:-120}"
    hold="${ARM_KEY6_AIRBORNE_HOLD_S:-8}"
    echo "AIRBORNE_DEMO: folded -> nose-forward straight with zero base yaw, ${duration}s extension, ${hold}s hold, ${return_duration}s damped return"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_DEMO_REFUSED: vehicle did not satisfy the stability gate"
      discard_buffered_keys
      return 1
    fi
  else
    echo "GROUND_DEMO_PREVIEW: folded -> nose-forward straight -> folded"
  fi
  echo "VISIBLE_DEMO_BEGIN: unfold toward ROS FLU +X / nose; shoulder_pan stays fixed"
  if ! run_preset flight_straight_forward "${duration}" 0.02 "${arming_state}"; then
    echo "VISIBLE_DEMO_ABORTED phase=extend reason=trajectory_command_failed"
    discard_buffered_keys
    return 1
  fi
  sleep "${hold}"
  if ! run_preset retracted "${return_duration}" 0.02 "${arming_state}"; then
    echo "VISIBLE_DEMO_ABORTED phase=retract reason=trajectory_command_failed"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    echo "AIRBORNE_DEMO_SETTLING: waiting for measured velocity stability"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_DEMO_UNSTABLE_AFTER_RETURN"
      discard_buffered_keys
      return 1
    fi
  fi
  echo "VISIBLE_DEMO_COMPLETE: ready for another cycle"
  discard_buffered_keys
}

run_gripper_demo() {
  local duration=5
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "GRIPPER_DEMO_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    duration=10
    echo "AIRBORNE_GRIPPER: zero-velocity gate, then slow 10s open/close"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_GRIPPER_REFUSED: vehicle did not satisfy the stability gate"
      discard_buffered_keys
      return 1
    fi
  fi
  ros2 run drone_arm_sim gripper_demo --open 1.2 --duration "${duration}" --hold 2
  discard_buffered_keys
}

run_cartesian_velocity_mode() {
  if ! ros2 topic list 2>/dev/null | grep -qx /my_drone/arm_cartesian_velocity_command; then
    echo "CARTESIAN_VELOCITY_REFUSED: persistent server is unavailable"
    return 1
  fi
  local arming_state=""
  if ! arming_state="$(get_vehicle_arming_state)"; then
    echo "CARTESIAN_VELOCITY_REFUSED: PX4 arming state is unavailable"
    discard_buffered_keys
    return 1
  fi
  if [[ "${arming_state}" == "armed" ]]; then
    echo "CARTESIAN_VELOCITY_GATE: requesting hover before endpoint control"
    if ! request_hover_and_wait_stable; then
      echo "CARTESIAN_VELOCITY_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  # One foreground ROS node owns one publisher for the complete V-mode
  # session.  It waits for the velocity server's subscription before putting
  # this terminal into single-key mode and restores the terminal on exit.
  ros2 run drone_arm_sim cartesian_arm_velocity_keyboard
  echo "CARTESIAN_VELOCITY_MODE_EXIT"
  discard_buffered_keys
}

echo "SO101 keyboard controller"
echo "Active profile: ${profile_label}"
echo "1 flight_work_a | 2 flight_work_b | 3/0 retracted"
echo "4 work_a (diagnostic) | 5 work_b (diagnostic)"
echo "6 FLIGHT DEMO: folded -> nose-forward straight -> folded | X exit"
echo "7 GRIPPER DEMO | 8 open | 9 close | X exit"
echo "V Cartesian endpoint velocity mode (I/K/J/L/U/O, SPACE: jerk-limited stop)"
echo "Manual jog (+/-): Q/A shoulder_pan | W/S shoulder_lift | E/D elbow_flex"
echo "                  R/F wrist_flex  | T/G wrist_roll    | Y/H gripper"
echo "Each press is one bounded smooth step; holding a key does not accumulate repeats."
echo "Use 1/2/3 during flight. Full 4/5 poses are ground-only."
echo "Key 6 starts from the original folded pose; in flight it defaults to"
echo "90s nose-forward extension, 8s hold, then a 120s low-disturbance return."

while true; do
  IFS= read -rsn1 key
  case "${key,,}" in
    1) run_flight_preset flight_work_a 12 ;;
    2) run_flight_preset flight_work_b 12 ;;
    3) run_flight_preset retracted 12 ;;
    0) run_flight_preset retracted 12 ;;
    4) run_ground_preset work_a 30 ;;
    5) run_ground_preset work_b 30 ;;
    6) if ! run_visible_demo; then
         echo "KEY6_DEMO_FAILED: controller remains open; inspect ${runtime_dir}/arm_keyboard.log"
       fi ;;
    7) run_gripper_demo ;;
    8) run_gripper_preset gripper_open ;;
    9) run_gripper_preset gripper_closed ;;
    v) run_cartesian_velocity_mode ;;
    q) run_jog shoulder_pan 0.05 ;;
    a) run_jog shoulder_pan -0.05 ;;
    w) run_jog shoulder_lift 0.05 ;;
    s) run_jog shoulder_lift -0.05 ;;
    e) run_jog elbow_flex 0.05 ;;
    d) run_jog elbow_flex -0.05 ;;
    r) run_jog wrist_flex 0.05 ;;
    f) run_jog wrist_flex -0.05 ;;
    t) run_jog wrist_roll 0.05 ;;
    g) run_jog wrist_roll -0.05 ;;
    y) run_jog gripper 0.10 ;;
    h) run_jog gripper -0.10 ;;
    x) break ;;
  esac
done
