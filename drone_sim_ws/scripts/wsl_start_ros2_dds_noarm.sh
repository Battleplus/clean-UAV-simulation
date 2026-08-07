#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
runtime_dir="/tmp/my_drone_ros2_dds"
mkdir -p "${runtime_dir}"

# A second launch would create another direct-wrench node subscribing to the
# same actuator topic and apply thrust twice to one Gazebo entity.
if [[ "${CLEAN_STALE_RUNTIME:-1}" == "1" ]]; then
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -x px4 2>/dev/null || true
  pkill -x MicroXRCEAgent 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  # ROS launch and the DDS keyboard controller both have process name ros2.
  pkill -x ros2 2>/dev/null || true
  sleep 1
fi

source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
ros2_controllers_overlay="${workspace_dir}/.deps/ros2_controllers_debs/overlay/opt/ros/jazzy"
if [[ -d "${ros2_controllers_overlay}" ]]; then
  export AMENT_PREFIX_PATH="${ros2_controllers_overlay}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${ros2_controllers_overlay}/lib:${LD_LIBRARY_PATH:-}"
fi
source "${workspace_dir}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

agent_log="${runtime_dir}/agent.log"
gazebo_log="${runtime_dir}/gazebo.log"
px4_log="${runtime_dir}/px4.log"
: >"${agent_log}"
: >"${gazebo_log}"
: >"${px4_log}"

setsid /home/asus/.local/bin/MicroXRCEAgent udp4 -p 8888 -v 4 \
  >"${agent_log}" 2>&1 &
echo $! >"${runtime_dir}/agent.pid"

setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:="${HEADLESS:-false}" enable_controller:=false \
  enable_arm_control:="${ENABLE_ARM_CONTROL:-false}" spawn_z:=1.0 \
  reaction_moment_ratio_m:="${REACTION_MOMENT_RATIO_M:--1}" \
  config_file:="${workspace_dir}/src/drone_arm_sim/config/my_drone_v2_cad_flight_pitch_corrected.json" \
  >"${gazebo_log}" 2>&1 &
echo $! >"${runtime_dir}/gazebo.pid"

for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/model/my_drone/link/base_link/sensor/imu_sensor/imu'; then
    break
  fi
  sleep 1
done

cd "${px4_dir}"
setsid env PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone PX4_SYS_AUTOSTART=4025 \
  ./build/px4_sitl_default/bin/px4 -d >"${px4_log}" 2>&1 &
echo $! >"${runtime_dir}/px4.pid"

for _ in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -Eq '^/fmu/out/vehicle_status(_v[0-9]+)?$'; then
    echo "ROS2_DDS_NOARM_READY"
    echo "ROS2_DDS_READY arm_control=${ENABLE_ARM_CONTROL:-false}"
    echo "logs=${runtime_dir}"
    exit 0
  fi
  sleep 1
done

echo "ROS2 DDS topics did not appear within 90 seconds" >&2
tail -n 80 "${agent_log}" >&2 || true
tail -n 80 "${px4_log}" >&2 || true
exit 1
