#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"

# Keep flight and arm input windows visually distinct.
printf '\033]0;my_drone 4kg 联合调试 - SO101 机械臂\007'

run_preset() {
  local preset="$1"
  local duration="$2"
  echo "Sending ${preset} (${duration}s)..."
  ros2 run drone_arm_sim arm_preset_control \
    --preset "${preset}" --duration "${duration}" --wait --tolerance 0.08
}

run_ground_preset() {
  if vehicle_is_armed; then
    echo "GROUND_ONLY_REFUSED: full diagnostic poses 4/5 are disabled in flight"
    discard_buffered_keys
    return 1
  fi
  run_preset "$1" "$2"
  discard_buffered_keys
}

vehicle_is_armed() {
  timeout 3 ros2 topic echo --once /fmu/out/vehicle_status_v4 2>/dev/null \
    | grep -q '^arming_state: 2'
}

request_hover_and_wait_stable() {
  echo "AIRBORNE_STABILITY_GATE: requesting H-equivalent hover and waiting for 5 stable seconds"
  ros2 topic pub --once /my_drone/hover_request std_msgs/msg/Bool '{data: true}' >/dev/null
  python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
    --horizontal 0.10 --vertical 0.08 --hold 5 --timeout 90
}

prepare_keyboard_motion() {
  if ! vehicle_is_armed; then
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
  if vehicle_is_armed; then
    echo "AIRBORNE_PRESET_GATE: requesting zero velocity before ${preset}"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_PRESET_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  run_preset "${preset}" "${duration}"
  discard_buffered_keys
}

run_jog() {
  local joint="$1"
  local delta="$2"
  local duration=1.5
  if vehicle_is_armed; then
    duration=2.5
    if ! prepare_keyboard_motion; then
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
  if vehicle_is_armed; then
    duration=6
    if ! prepare_keyboard_motion; then
      echo "AIRBORNE_GRIPPER_REFUSED: aircraft is not in a stable hover"
      discard_buffered_keys
      return 1
    fi
  fi
  run_preset "${preset}" "${duration}"
  discard_buffered_keys
}

discard_buffered_keys() {
  local ignored
  # Keys pressed while a trajectory is running remain queued in the terminal.
  # Discard them so repeatedly tapping 6 cannot silently launch another full
  # cycle as soon as the first one returns.
  while IFS= read -rsn1 -t 0.01 ignored; do :; done
}

run_visible_demo() {
  local duration=8
  local distance=0.12
  local hold=3
  if vehicle_is_armed; then
    duration="${ARM_KEY6_AIRBORNE_DURATION_S:-90}"
    distance="${ARM_KEY6_AIRBORNE_DISTANCE_M:-0.10}"
    hold="${ARM_KEY6_AIRBORNE_HOLD_S:-8}"
    echo "AIRBORNE_DEMO: using ${distance}m, ${duration}s each way and ${hold}s extended hold"
    if ! request_hover_and_wait_stable; then
      echo "AIRBORNE_DEMO_REFUSED: vehicle did not satisfy the stability gate"
      discard_buffered_keys
      return 1
    fi
  else
    echo "GROUND_DEMO: using 8s extend/retract trajectories"
  fi
  echo "VISIBLE_DEMO_BEGIN: Cartesian tool-forward extension and exact-path return"
  ros2 run drone_arm_sim cartesian_arm_demo \
    --distance "${distance}" --step 0.005 --duration "${duration}" --hold "${hold}"
  if vehicle_is_armed; then
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
  if vehicle_is_armed; then
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

echo "SO101 keyboard controller"
echo "1 flight_work_a | 2 flight_work_b | 3/0 retracted"
echo "4 work_a (diagnostic) | 5 work_b (diagnostic)"
echo "6 CARTESIAN DEMO: tool-forward 0.12m -> hold -> exact-path return | X exit"
echo "7 GRIPPER DEMO | 8 open | 9 close | X exit"
echo "Manual jog (+/-): Q/A shoulder_pan | W/S shoulder_lift | E/D elbow_flex"
echo "                  R/F wrist_flex  | T/G wrist_roll    | Y/H gripper"
echo "Each press is one bounded smooth step; holding a key does not accumulate repeats."
echo "Use 1/2/3 during flight. Full 4/5 poses are ground-only."
echo "Key 6 uses 0.12m on ground; in flight it defaults to 0.10m/90s each way,"
echo "an 8s extended hold, then a measured-velocity stability check."

while true; do
  IFS= read -rsn1 key
  case "${key,,}" in
    1) run_flight_preset flight_work_a 12 ;;
    2) run_flight_preset flight_work_b 12 ;;
    3) run_flight_preset retracted 12 ;;
    0) run_flight_preset retracted 12 ;;
    4) run_ground_preset work_a 30 ;;
    5) run_ground_preset work_b 30 ;;
    6) run_visible_demo ;;
    7) run_gripper_demo ;;
    8) run_gripper_preset gripper_open ;;
    9) run_gripper_preset gripper_closed ;;
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
