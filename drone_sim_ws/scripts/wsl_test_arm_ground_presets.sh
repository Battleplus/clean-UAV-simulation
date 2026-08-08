#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_log="${workspace_dir}/analysis/cad_direct/arm_ground_presets.log"
overlay="${workspace_dir}/.deps/ros2_controllers_debs/overlay/opt/ros/jazzy"

source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
if [[ -d "${overlay}" ]]; then
  export AMENT_PREFIX_PATH="${overlay}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${overlay}/lib:/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
fi

# ROS setup scripts intentionally read optional variables that may be unset;
# enable nounset only after all overlays have been sourced.
set -u

pkill -x gazebo_direct_m 2>/dev/null || true
pkill -x gazebo_sensor_d 2>/dev/null || true
pkill -x parameter_bridg 2>/dev/null || true
pkill -x robot_state_pub 2>/dev/null || true
pkill -f '/arm_coupling_monitor' 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -x MicroXRCEAgent 2>/dev/null || true
pkill -x ruby 2>/dev/null || true
pkill -x ros2 2>/dev/null || true
sleep 1

export MY_DRONE_URDF="${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
exec >"${runtime_log}" 2>&1
ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true enable_controller:=false enable_arm_control:=true \
  spawn_z:=1.0 \
  config_file:="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json" &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x gazebo_sensor_d 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  pkill -x ros2 2>/dev/null || true
}
trap cleanup EXIT
sleep 45

for preset in retracted work_a work_b flight_work_a flight_work_b retracted; do
  echo "ARM_GROUND_SENT_${preset}"
  ros2 run drone_arm_sim arm_preset_control \
    --preset "${preset}" --duration 5 --wait --tolerance 0.08
done

echo "ARM_GROUND_PRESETS_PASS"
