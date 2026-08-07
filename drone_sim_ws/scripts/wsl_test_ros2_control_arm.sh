#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
overlay="${workspace_dir}/.deps/ros2_controllers_debs/overlay/opt/ros/jazzy"
log_file="/tmp/my_drone_ros2_control_arm.log"

cleanup() {
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -x ros2 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 1

source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="${overlay}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:${overlay}/lib:/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
source "${workspace_dir}/install/setup.bash"
set -u

: >"${log_file}"
setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true enable_controller:=false enable_arm_control:=true spawn_z:=0.8 \
  >"${log_file}" 2>&1 &

for _ in $(seq 1 45); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  sleep 1
done

if ! ros2 service list | grep -q '^/controller_manager/list_controllers$'; then
  echo "ARM_ROS2_CONTROL_MANAGER_MISSING" >&2
  tail -n 160 "${log_file}" >&2
  exit 1
fi

for _ in $(seq 1 30); do
  controllers="$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null || true)"
  if grep -q 'arm_controller.*active' <<<"${controllers}" && \
     grep -q 'joint_state_broadcaster.*active' <<<"${controllers}"; then
    break
  fi
  sleep 1
done

controllers="$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}')"
echo "${controllers}"
grep -q 'arm_controller.*active' <<<"${controllers}"
grep -q 'joint_state_broadcaster.*active' <<<"${controllers}"

ros2 run drone_arm_sim arm_preset_control \
  --preset work_a --duration 3 --wait --tolerance 0.04

joint_state="$(timeout 8 ros2 topic echo --once /joint_states)"
echo "${joint_state}"
for joint in shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper; do
  grep -q -- "- ${joint}" <<<"${joint_state}"
done

ros2 run drone_arm_sim arm_preset_control \
  --preset retracted --duration 3 --wait --tolerance 0.04
echo "ARM_ROS2_CONTROL_GROUND_PASS"
echo "log=${log_file}"
