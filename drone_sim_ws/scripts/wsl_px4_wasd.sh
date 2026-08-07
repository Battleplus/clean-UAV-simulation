#!/usr/bin/env bash
set -eo pipefail
trap 'status=$?; echo "WASD launcher failed at line ${LINENO}: ${BASH_COMMAND} (exit ${status})" >&2' ERR

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframe_id="${AIRFRAME_ID:-4025}"
config_file="${CONFIG_FILE:-${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/config/my_drone_v2_cad_flight_pitch_corrected.json}"
use_bootstrap="${USE_BOOTSTRAP:-0}"
gazebo_log="/tmp/my_drone_px4_wasd_gazebo.log"
px4_log="/tmp/my_drone_px4_wasd.log"
bootstrap_log="/tmp/my_drone_px4_wasd_bootstrap.log"
gui_log="/tmp/my_drone_px4_wasd_gui.log"
fifo="/tmp/my_drone_px4_wasd_fifo_$$"

source /opt/ros/jazzy/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install
source install/setup.bash

mkfifo "${fifo}"
exec 3<>"${fifo}"

clear_bootstrap_wrench() {
  gz topic \
    -t /world/flight_world/wrench/clear \
    -m gz.msgs.Entity \
    -p 'name: "my_drone::base_link", type: LINK' \
    >/dev/null 2>&1 || true
}

cleanup() {
  printf 'shutdown\n' >&3 2>/dev/null || true
  if [[ -n "${px4_pid:-}" ]]; then
    kill -- "-${px4_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${launch_pid:-}" ]]; then
    kill -- "-${launch_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${gui_pid:-}" ]]; then
    kill -- "-${gui_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${bootstrap_pid:-}" ]]; then
    kill -INT -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
    clear_bootstrap_wrench
  fi
  exec 3>&-
  rm -f "${fifo}"
}
trap cleanup EXIT

setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  spawn_z:=0.45 headless:=true enable_controller:=false config_file:="${config_file}" \
  >"${gazebo_log}" 2>&1 &
launch_pid=$!
if [[ "${use_bootstrap}" -eq 1 ]]; then
  sleep 3
  setsid ros2 run drone_arm_sim gazebo_wrench_controller \
    --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v2/my_drone_cad_dynamic.urdf" \
    --target 0 0 0.45 \
    --entity-name "my_drone::base_link" \
    --persistent-bootstrap \
    >"${bootstrap_log}" 2>&1 &
  bootstrap_pid=$!
else
  sleep 15
fi

ready=0
for attempt in 1 2 3; do
  echo "启动 PX4（第 ${attempt}/3 次）..."
  : >"${px4_log}"
  cd "${px4_dir}"
  setsid env \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD=flight_world \
    PX4_GZ_MODEL_NAME=my_drone \
    PX4_SYS_AUTOSTART="${airframe_id}" \
    ./build/px4_sitl_default/bin/px4 \
    <"${fifo}" >"${px4_log}" 2>&1 &
  px4_pid=$!

  for _ in $(seq 1 60); do
    if grep -q "Ready for takeoff" "${px4_log}"; then
      ready=1
      break
    fi
    if ! kill -0 "${px4_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  if [[ "${ready}" -eq 1 ]]; then
    break
  fi
  echo "本次 EKF 未就绪，自动重启 PX4..."
  printf 'shutdown\n' >&3
  wait "${px4_pid}" 2>/dev/null || true
  px4_pid=""
  sleep 2
done
if [[ "${ready}" -ne 1 ]]; then
  echo "PX4 连续三次未就绪；请查看 ${px4_log}。" >&2
  exit 1
fi

if [[ "${use_bootstrap}" -eq 1 ]]; then
  timeout 30 ros2 run drone_arm_sim hover_acceptance \
    --target 0 0 0.45 \
    --settle-time 3 \
    --position-tolerance 0.20 \
    --attitude-tolerance-deg 5.0 \
    >/tmp/my_drone_px4_wasd_bootstrap_acceptance.log 2>&1 || \
    echo "Bootstrap stabilization check warned; continuing to PX4 control."
fi

printf '%s\n' \
  "param set NAV_DLL_ACT 0" \
  "param set MC_YAW_P 1.0" \
  "param set MC_YAW_WEIGHT 0.3" \
  "param set MC_YAWRATE_K 0.3" >&3
sleep 2

export GZ_SIM_RESOURCE_PATH="${workspace_dir}/install/drone_arm_sim/share"
setsid gz sim -g -v 1 >"${gui_log}" 2>&1 &
gui_pid=$!
sleep 3

if [[ -n "${bootstrap_pid:-}" ]]; then
  kill -INT -- "-${bootstrap_pid}" >/dev/null 2>&1 || true
  wait "${bootstrap_pid}" 2>/dev/null || true
  clear_bootstrap_wrench
  bootstrap_pid=""
fi
sleep 1

cd "${workspace_dir}"
"${px4_dir}/.venv/bin/python" \
  "${workspace_dir}/scripts/px4_wasd_control.py"

echo "等待降落..."
sleep 8
