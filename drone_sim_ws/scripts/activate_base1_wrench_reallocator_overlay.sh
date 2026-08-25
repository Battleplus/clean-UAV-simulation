#!/usr/bin/env bash
# Replace only the Base 1 direct-motor subscriber with the optional pre-allocation overlay.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="/tmp/my_drone_ros2_dds"
cpu_role_runner="${workspace_dir}/scripts/run_with_cpu_role.sh"
config_file="${CONFIG_FILE:-${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json}"
expected_mass_kg="${ARM_COUPLING_TARGET_MASS_KG:-4.0}"
output_topic="/my_drone/base1_compensated/command/motor_speed"
mkdir -p "${runtime_dir}"

# A `setsid ros2 run ...` PID identifies the ros2 CLI wrapper, while the
# installed node is its child in the same process group.  Validate the saved
# command before signalling the whole group so a stale/reused PID can never
# terminate an unrelated process.
stop_saved_ros2_session() {
  local pid_file="$1" required_fragment="$2"
  local saved_pid="" command_line="" pgid=""
  [[ -f "${pid_file}" ]] || return 0
  saved_pid="$(tr -cd '0-9' <"${pid_file}")"
  if [[ -n "${saved_pid}" ]] && kill -0 "${saved_pid}" 2>/dev/null; then
    command_line="$(ps -o args= -p "${saved_pid}" 2>/dev/null || true)"
    if [[ "${command_line}" == *"${required_fragment}"* ]]; then
      pgid="$(ps -o pgid= -p "${saved_pid}" 2>/dev/null | tr -d ' ' || true)"
      if [[ -n "${pgid}" && "${pgid}" == "${saved_pid}" ]]; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
      else
        kill -TERM "${saved_pid}" 2>/dev/null || true
      fi
    fi
  fi
  rm -f "${pid_file}"
}

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

python3 - "${config_file}" "${expected_mass_kg}" <<'PY'
import json, pathlib, sys
config = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
actual = float(config.get("estimated_all_up_mass_kg", -1.0))
expected = float(sys.argv[2])
if abs(actual - expected) > 1.0e-9:
    raise SystemExit(
        f"REFUSED: overlay mass guard mismatch config={actual} expected={expected}"
    )
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

stop_saved_ros2_session \
  "${runtime_dir}/base1_reallocator.pid" \
  "ros2 run drone_arm_sim base1_wrench_reallocator"
stop_saved_ros2_session \
  "${runtime_dir}/direct_xy_guardian.pid" \
  "direct_xy_guardian"
# Also remove an orphan whose PID file was lost; the bracketed executable
# pattern avoids matching this shell command itself.
pkill -f '/[b]ase1_wrench_reallocator' 2>/dev/null || true

reallocator_args=(
  --config "${config_file}"
  --expected-mass-kg "${expected_mass_kg}"
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
  --adaptive-baseline-hold-s "${BASE1_ADAPTIVE_BASELINE_HOLD_S:-5.0}"
  --adaptive-time-constant-s "${BASE1_ADAPTIVE_TIME_CONSTANT_S:-15.0}"
  --adaptive-leak-time-constant-s "${BASE1_ADAPTIVE_LEAK_TIME_CONSTANT_S:-60.0}"
  --adaptive-warmup-s "${BASE1_ADAPTIVE_WARMUP_S:-3.0}"
  --adaptive-force-deadband-n "${BASE1_ADAPTIVE_FORCE_DEADBAND_N:-0.02}"
  --adaptive-torque-deadband-nm "${BASE1_ADAPTIVE_TORQUE_DEADBAND_NM:-0.005}"
  --adaptive-horizontal-force-limit-n "${BASE1_ADAPTIVE_HORIZONTAL_FORCE_LIMIT_N:-0.06}"
  --adaptive-vertical-force-limit-n "${BASE1_ADAPTIVE_VERTICAL_FORCE_LIMIT_N:-0.04}"
  --adaptive-torque-limit-nm "${BASE1_ADAPTIVE_TORQUE_LIMIT_NM:-0.02}"
  --adaptive-horizontal-force-rate-n-s "${BASE1_ADAPTIVE_HORIZONTAL_FORCE_RATE_N_S:-0.005}"
  --adaptive-vertical-force-rate-n-s "${BASE1_ADAPTIVE_VERTICAL_FORCE_RATE_N_S:-0.003}"
  --adaptive-torque-rate-nm-s "${BASE1_ADAPTIVE_TORQUE_RATE_NM_S:-0.002}"
  --adaptive-joint-velocity-limit-rad-s "${BASE1_ADAPTIVE_JOINT_VELOCITY_LIMIT_RAD_S:-0.005}"
  --adaptive-joint-acceleration-limit-rad-s2 "${BASE1_ADAPTIVE_JOINT_ACCELERATION_LIMIT_RAD_S2:-0.02}"
  --adaptive-body-speed-limit-m-s "${BASE1_ADAPTIVE_BODY_SPEED_LIMIT_M_S:-0.03}"
  --adaptive-angular-rate-limit-rad-s "${BASE1_ADAPTIVE_ANGULAR_RATE_LIMIT_RAD_S:-0.008726646}"
  --adaptive-update-residual-limit "${BASE1_ADAPTIVE_UPDATE_RESIDUAL_LIMIT:-0.005}"
  --diagnostic-rate-hz "${BASE1_DIAGNOSTIC_RATE_HZ:-100.0}"
)
if [[ "${BASE1_COMPENSATION_ENABLED:-false}" == "true" ]]; then
  reallocator_args+=(--enabled)
fi
if [[ "${BASE1_POSITION_FEEDBACK_ENABLED:-false}" == "true" ]]; then
  reallocator_args+=(--position-feedback-enabled)
fi
if [[ "${BASE1_ADAPTIVE_ENABLED:-false}" == "true" ]]; then
  reallocator_args+=(--adaptive-enabled)
fi
setsid "${cpu_role_runner}" support \
  ros2 run drone_arm_sim base1_wrench_reallocator "${reallocator_args[@]}" \
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

# Candidate mixed-axis ownership uses a process-isolated guardian.  It starts
# before the interactive controller so the latter can establish a fresh idle
# session before any arm-motion edge.  The guardian is the only force-command
# and authoritative owner-state writer in this mode.
if [[ "${ENABLE_ARM_CONTROL:-false}" == "true" \
  && "${ARM_DIRECT_XY_OWNERSHIP:-false}" == "true" \
  && "${ARM_DIRECT_XY_EXTERNAL_GUARDIAN:-false}" == "true" ]]; then
  # Run the module source from this exact workspace.  `ros2 run` can retain an
  # older executable index across a just-completed colcon install, while the
  # generated console wrapper can independently fail when stale Python
  # distribution metadata is visible.  Neither cache is part of the guardian
  # safety contract, so formal acceptance bypasses both and verifies the
  # authoritative source path explicitly.
  guardian_source="${workspace_dir}/src/px4_ros2_control/px4_ros2_control/direct_xy_guardian.py"
  if [[ ! -f "${guardian_source}" ]]; then
    echo "Direct-XY guardian source is missing: ${guardian_source}" >&2
    exit 1
  fi
  setsid "${cpu_role_runner}" guardian python3 "${guardian_source}" \
    >"${runtime_dir}/direct_xy_guardian.log" 2>&1 &
  guardian_pid=$!
  echo "${guardian_pid}" >"${runtime_dir}/direct_xy_guardian.pid"
  for _ in $(seq 1 50); do
    if grep -q 'DIRECT_XY_GUARDIAN_READY' "${runtime_dir}/direct_xy_guardian.log" 2>/dev/null; then
      break
    fi
    if ! kill -0 "${guardian_pid}" 2>/dev/null; then
      cat "${runtime_dir}/direct_xy_guardian.log" >&2 || true
      exit 1
    fi
    sleep 0.1
  done
  if ! grep -q 'DIRECT_XY_GUARDIAN_READY' "${runtime_dir}/direct_xy_guardian.log" 2>/dev/null; then
    cat "${runtime_dir}/direct_xy_guardian.log" >&2 || true
    echo "Direct-XY guardian did not receive three fresh physical reports" >&2
    exit 1
  fi
fi

# The original frozen launch starts exactly one direct-motor process.  Stop
# that subscriber only after the replacement command stream exists.  The
# aircraft must still be disarmed/on the ground when this overlay is enabled.
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
setsid "${cpu_role_runner}" bulk \
  ros2 run drone_arm_sim gazebo_direct_motor_model "${motor_args[@]}" \
  >"${runtime_dir}/base1_overlay_motor.log" 2>&1 &
motor_pid=$!
echo "${motor_pid}" >"${runtime_dir}/base1_overlay_motor.pid"

sleep 1
if ! kill -0 "${motor_pid}" 2>/dev/null; then
  cat "${runtime_dir}/base1_overlay_motor.log" >&2 || true
  echo "Base 1 overlay motor failed; refusing to arm" >&2
  exit 1
fi

# Preload the Cartesian velocity planner while the vehicle is still on the
# ground.  Flight-time keys then send only tiny String messages; they never
# start an IK/preflight Python process in the sensor-critical flight window.
if [[ "${BASE1_CARTESIAN_VELOCITY_SERVER_ENABLED:-true}" == "true" ]]; then
  stop_saved_ros2_session \
    "${runtime_dir}/cartesian_velocity.pid" \
    "ros2 run drone_arm_sim cartesian_arm_velocity_control"
  pkill -f '/[c]artesian_arm_velocity_control' 2>/dev/null || true
  setsid "${cpu_role_runner}" bulk env \
    SO101_MOTION_REFERENCE="${SO101_MOTION_REFERENCE:-${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference_4kg.json}" \
    MY_DRONE_FLIGHT_CONFIG="${CONFIG_FILE:-${config_file}}" \
    ros2 run drone_arm_sim cartesian_arm_velocity_control \
    --urdf "${SO101_KINEMATICS_URDF:-${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf}" \
    --speed "${BASE1_CARTESIAN_SPEED_M_S:-0.010}" \
    --acceleration "${BASE1_CARTESIAN_ACCELERATION_M_S2:-0.010}" \
    --jerk "${BASE1_CARTESIAN_JERK_M_S3:-0.020}" \
    --horizon "${BASE1_CARTESIAN_HORIZON_S:-1.0}" \
    >"${runtime_dir}/cartesian_velocity.log" 2>&1 &
  cartesian_velocity_pid=$!
  echo "${cartesian_velocity_pid}" >"${runtime_dir}/cartesian_velocity.pid"
  cartesian_velocity_ready=false
  for _ in $(seq 1 60); do
    if ! kill -0 "${cartesian_velocity_pid}" 2>/dev/null; then
      cat "${runtime_dir}/cartesian_velocity.log" >&2 || true
      echo "Base1 Cartesian velocity server exited during startup" >&2
      exit 1
    fi
    if ros2 topic list 2>/dev/null | grep -qx /my_drone/arm_cartesian_velocity_command; then
      cartesian_velocity_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "${cartesian_velocity_ready}" != "true" ]]; then
    echo "Base1 Cartesian velocity command topic did not appear" >&2
    exit 1
  fi
  echo "BASE1_CARTESIAN_VELOCITY_READY pid=${cartesian_velocity_pid}"
fi

echo "BASE1_WRENCH_OVERLAY_READY enabled=${BASE1_COMPENSATION_ENABLED:-false}"
echo "input=/my_drone/command/motor_speed output=${output_topic}"
echo "logs=${runtime_dir}/base1_reallocator.log ${runtime_dir}/base1_overlay_motor.log"
