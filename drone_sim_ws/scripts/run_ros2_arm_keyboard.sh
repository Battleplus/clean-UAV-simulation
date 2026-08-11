#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"

run_preset() {
  local preset="$1"
  local duration="$2"
  echo "Sending ${preset} (${duration}s)..."
  ros2 run drone_arm_sim arm_preset_control \
    --preset "${preset}" --duration "${duration}" --wait --tolerance 0.08
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
  if vehicle_is_armed; then
    duration=30
    distance=0.10
    echo "AIRBORNE_DEMO: using 0.10m and slower 30s extend/retract trajectories"
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
    --distance "${distance}" --step 0.005 --duration "${duration}" --hold 3
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
echo "1 flight_work_a | 2 flight_work_b | 3 retracted"
echo "4 work_a (diagnostic) | 5 work_b (diagnostic)"
echo "6 CARTESIAN DEMO: tool-forward 0.12m -> hold -> exact-path return | X exit"
echo "7 GRIPPER DEMO: open -> hold -> return | X exit"
echo "Use 1/2/3 during flight. Full 4/5 poses are ground-only."
echo "Key 6 uses 0.12m on ground; in flight it uses a safer 0.10m/30s path plus a 10s settle."

while true; do
  IFS= read -rsn1 key
  case "${key,,}" in
    1) run_preset flight_work_a 12 ;;
    2) run_preset flight_work_b 12 ;;
    3) run_preset retracted 12 ;;
    4) run_preset work_a 30 ;;
    5) run_preset work_b 30 ;;
    6) run_visible_demo ;;
    7) run_gripper_demo ;;
    x) break ;;
  esac
done
