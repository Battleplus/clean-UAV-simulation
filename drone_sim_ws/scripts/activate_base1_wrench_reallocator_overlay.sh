#!/usr/bin/env bash
# Replace only the Base 1 direct-motor subscriber with the optional pre-allocation overlay.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="/tmp/my_drone_ros2_dds"
config_file="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json"
output_topic="/my_drone/base1_compensated/command/motor_speed"
mkdir -p "${runtime_dir}"

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

python3 - "${config_file}" <<'PY'
import json, pathlib, sys
config = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if float(config.get("estimated_all_up_mass_kg", -1.0)) != 4.0:
    raise SystemExit("REFUSED: Base 1 overlay accepts only the frozen 4.0 kg config")
PY

if ! ros2 topic list 2>/dev/null | grep -qx /my_drone/command/motor_speed; then
  echo "REFUSED: Base 1 raw motor topic is unavailable; start Base 1 first" >&2
  exit 1
fi
if ! ros2 topic list 2>/dev/null | grep -qx /model/my_drone/odometry; then
  echo "REFUSED: Base 1 Gazebo odometry is unavailable" >&2
  exit 1
fi

# The estimator is an independent, read-only 100 Hz overlay and refuses to
# start unless the Base 1 arm joint stream exists.
"${workspace_dir}/scripts/run_base1_readonly_estimator_overlay.sh"

if [[ -f "${runtime_dir}/base1_reallocator.pid" ]]; then
  prior="$(cat "${runtime_dir}/base1_reallocator.pid")"
  kill "${prior}" 2>/dev/null || true
fi
# Also remove an orphan whose PID file was lost; the bracketed executable
# pattern avoids matching this shell command itself.
pkill -f '/[b]ase1_wrench_reallocator' 2>/dev/null || true

reallocator_args=(
  --config "${config_file}"
  --input-topic /my_drone/command/motor_speed
  --output-topic "${output_topic}"
  --source-timeout-s "${BASE1_COMP_SOURCE_TIMEOUT_S:-0.50}"
  --flight-state-timeout-s "${BASE1_COMP_FLIGHT_STATE_TIMEOUT_S:-5.0}"
  --reaction-force-gain "${BASE1_REACTION_FORCE_GAIN:-0.0}"
  --reaction-torque-gain "${BASE1_REACTION_TORQUE_GAIN:-0.0}"
  --gravity-torque-gain "${BASE1_GRAVITY_TORQUE_GAIN:-0.0}"
  --force-limit-n "${BASE1_COMP_FORCE_LIMIT_N:-1.0}"
  --reaction-torque-limit-nm "${BASE1_REACTION_TORQUE_LIMIT_NM:-0.10}"
  --gravity-torque-limit-nm "${BASE1_GRAVITY_TORQUE_LIMIT_NM:-0.10}"
  --force-slew-rate-n-s "${BASE1_COMP_FORCE_SLEW_N_S:-1.0}"
  --torque-slew-rate-nm-s "${BASE1_COMP_TORQUE_SLEW_NM_S:-0.10}"
  --maximum-motor-delta-n "${BASE1_COMP_MAX_MOTOR_DELTA_N:-0.50}"
  --minimum-headroom-n "${BASE1_COMP_MIN_HEADROOM_N:-0.25}"
  --maximum-residual-norm "${BASE1_COMP_MAX_RESIDUAL_NORM:-0.02}"
  --truth-timeout-s "${BASE1_COMP_TRUTH_TIMEOUT_S:-0.50}"
  --arm-motion-timeout-s "${BASE1_COMP_ARM_MOTION_TIMEOUT_S:-1.0}"
  --position-gain-n-m "${BASE1_POSITION_GAIN_N_M:-4.0}"
  --velocity-gain-n-s-m "${BASE1_VELOCITY_GAIN_N_S_M:-2.0}"
  --position-horizontal-limit-n "${BASE1_POSITION_HORIZONTAL_LIMIT_N:-0.20}"
  --position-vertical-limit-n "${BASE1_POSITION_VERTICAL_LIMIT_N:-0.15}"
)
if [[ "${BASE1_COMPENSATION_ENABLED:-false}" == "true" ]]; then
  reallocator_args+=(--enabled)
fi
if [[ "${BASE1_POSITION_FEEDBACK_ENABLED:-false}" == "true" ]]; then
  reallocator_args+=(--position-feedback-enabled)
fi
setsid ros2 run drone_arm_sim base1_wrench_reallocator "${reallocator_args[@]}" \
  >"${runtime_dir}/base1_reallocator.log" 2>&1 &
reallocator_pid=$!
echo "${reallocator_pid}" >"${runtime_dir}/base1_reallocator.pid"

for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx "${output_topic}"; then
    break
  fi
  if ! kill -0 "${reallocator_pid}" 2>/dev/null; then
    cat "${runtime_dir}/base1_reallocator.log" >&2 || true
    exit 1
  fi
  sleep 0.2
done
if ! ros2 topic list 2>/dev/null | grep -qx "${output_topic}"; then
  echo "Base 1 reallocator output topic did not appear" >&2
  exit 1
fi

# ── Disarmed gate ──────────────────────────────────────────────────────
# The aircraft must be DISARMED before swapping the motor subscriber.
# Query PX4 VehicleStatus; refuse if armed or stale.
VEH_STATUS_TOPIC="/fmu/out/vehicle_status_v4"
MAX_STATUS_WAIT_S=3
_status_start=$(date +%s%N 2>/dev/null || echo 0)
_found_disarmed=false
for _ in $(seq 1 60); do
  # Read the latest VehicleStatus via ros2 topic echo (timeout 0.2s).
  _raw=$(timeout 0.2 ros2 topic echo --once "${VEH_STATUS_TOPIC}" 2>/dev/null || true)
  if echo "${_raw}" | grep -q 'ARMING_STATE_DISARMED'; then
    _found_disarmed=true
    break
  fi
  sleep 0.05
done
if [[ "${_found_disarmed}" != "true" ]]; then
  echo "REFUSED: VehicleStatus did not show DISARMED within ${MAX_STATUS_WAIT_S}s" >&2
  kill "${reallocator_pid}" 2>/dev/null || true
  exit 1
fi

# The original frozen launch starts exactly one direct-motor process.  Stop
# that subscriber only after the replacement command stream exists.
mapfile -t old_motor_pids < <(pgrep -x gazebo_direct_m || true)
if [[ "${#old_motor_pids[@]}" -ne 1 ]]; then
  echo "REFUSED: expected exactly one Base 1 direct-motor process, found ${#old_motor_pids[@]}" >&2
  kill "${reallocator_pid}" 2>/dev/null || true
  exit 1
fi
old_motor_pid="${old_motor_pids[0]}"
kill "${old_motor_pid}"
for _ in $(seq 1 30); do
  kill -0 "${old_motor_pid}" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "${old_motor_pid}" 2>/dev/null; then
  echo "REFUSED: original Base 1 direct-motor process did not stop" >&2
  kill "${reallocator_pid}" 2>/dev/null || true
  exit 1
fi

motor_args=(
  --config "${config_file}"
  --entity-name base_link
  --command-topic "${output_topic}"
  --reaction-moment-ratio-m "${REACTION_MOMENT_RATIO_M:-0.005}"
  --battery-dynamics-enabled false
  --arm-torque-feedforward-enabled false
  --arm-disturbance-observer-enabled false
)
setsid ros2 run drone_arm_sim gazebo_direct_motor_model "${motor_args[@]}" \
  >"${runtime_dir}/base1_overlay_motor.log" 2>&1 &
motor_pid=$!
echo "${motor_pid}" >"${runtime_dir}/base1_overlay_motor.pid"

sleep 1
if ! kill -0 "${motor_pid}" 2>/dev/null; then
  cat "${runtime_dir}/base1_overlay_motor.log" >&2 || true
  echo "Base 1 overlay motor failed; attempting rollback" >&2
  # Rollback: restart the original direct-motor subscriber.
  rollback_args=(
    --config "${config_file}"
    --entity-name base_link
    --command-topic /my_drone/command/motor_speed
    --reaction-moment-ratio-m "${REACTION_MOMENT_RATIO_M:-0.005}"
    --battery-dynamics-enabled false
    --arm-torque-feedforward-enabled false
    --arm-disturbance-observer-enabled false
  )
  setsid ros2 run drone_arm_sim gazebo_direct_motor_model "${rollback_args[@]}" \
    >"${runtime_dir}/base1_rollback_motor.log" 2>&1 &
  rollback_pid=$!
  echo "${rollback_pid}" >"${runtime_dir}/base1_rollback_motor.pid"
  sleep 1
  if kill -0 "${rollback_pid}" 2>/dev/null; then
    echo "Rollback: original motor subscriber restarted (pid ${rollback_pid})" >&2
  else
    echo "CRITICAL: rollback motor also failed" >&2
    cat "${runtime_dir}/base1_rollback_motor.log" >&2 || true
  fi
  kill "${reallocator_pid}" 2>/dev/null || true
  exit 1
fi

echo "BASE1_WRENCH_OVERLAY_READY enabled=${BASE1_COMPENSATION_ENABLED:-false}"
echo "input=/my_drone/command/motor_speed output=${output_topic}"
echo "logs=${runtime_dir}/base1_reallocator.log ${runtime_dir}/base1_overlay_motor.log"
