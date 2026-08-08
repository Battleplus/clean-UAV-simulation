#!/usr/bin/env bash
set -eo pipefail

# Controlled Gazebo contact regression for the formal CAD gripper.  This is
# deliberately separate from the PX4 flight world: four static corner stands
# support the dynamic CAD airframe at 1 m while a dynamic 0.25 kg cube rests
# on a narrow pedestal at the formal work_a end-effector position.

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
overlay="${workspace_dir}/.deps/ros2_controllers_debs/overlay/opt/ros/jazzy"
result_log="${workspace_dir}/analysis/cad_direct/grasp_contact_validation.log"
launch_log="/tmp/my_drone_grasp_contact_launch.log"
contact_raw="/tmp/my_drone_grasp_contacts.raw"

source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
if [[ -d "${overlay}" ]]; then
  export AMENT_PREFIX_PATH="${overlay}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${overlay}/lib:/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
fi

pkill -x gazebo_direct_m 2>/dev/null || true
pkill -x gazebo_motor_con 2>/dev/null || true
pkill -x gazebo_sensor_d 2>/dev/null || true
pkill -x parameter_bridg 2>/dev/null || true
pkill -x robot_state_pub 2>/dev/null || true
pkill -f '/arm_coupling_monitor' 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -x ruby 2>/dev/null || true
pkill -x ros2 2>/dev/null || true
sleep 1

robot="${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
world="${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/worlds/flight_world_grasp_test.sdf"
config="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json"
export MY_DRONE_URDF="${robot}"
export MY_DRONE_WORLD="${world}"
unset MY_DRONE_KINEMATIC_BASE
: >"${launch_log}"
: >"${contact_raw}"
: >"${result_log}"

setsid ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:=true enable_controller:=false enable_arm_control:=true \
  spawn_z:=1.0 target_ned_z:=-1.0 \
  config_file:="${config}" >"${launch_log}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -- "-${launch_pid}" 2>/dev/null || true
  pkill -x gazebo_direct_m 2>/dev/null || true
  pkill -x gazebo_motor_con 2>/dev/null || true
  pkill -x gazebo_sensor_d 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  pkill -x ros2 2>/dev/null || true
}
trap cleanup EXIT

contact_topic="/world/flight_world/model/grasp_test_object/link/link/sensor/contact_sensor/contact"
for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -qx "${contact_topic}" && \
     ros2 action list 2>/dev/null | grep -qx '/arm_controller/follow_joint_trajectory'; then
    break
  fi
  sleep 1
done
if ! gz topic -l 2>/dev/null | grep -qx "${contact_topic}"; then
  echo "GRASP_CONTACT_FAIL contact topic unavailable" | tee -a "${result_log}"
  tail -n 120 "${launch_log}" >>"${result_log}"
  exit 1
fi
if ! ros2 action list 2>/dev/null | grep -qx '/arm_controller/follow_joint_trajectory'; then
  echo "GRASP_CONTACT_FAIL arm controller unavailable" | tee -a "${result_log}"
  tail -n 120 "${launch_log}" >>"${result_log}"
  exit 1
fi

# Prove that the supported airframe remained at its requested pose, not the
# earlier free-fall state.  This is a fixture check, not a flight acceptance.
echo "GRASP_FIXTURE_SUPPORT_STAND=true" | tee -a "${result_log}"
timeout 30s ros2 run drone_arm_sim hover_acceptance \
  --target 0 0 1 --settle-time 8 \
  --position-tolerance 0.10 --attitude-tolerance-deg 8 \
  2>&1 | tee -a "${result_log}"

ros2 run drone_arm_sim arm_preset_control \
  --preset retracted --duration 5 --wait --tolerance 0.12 \
  2>&1 | tee -a "${result_log}"

timeout 18s gz topic -e -t "${contact_topic}" >"${contact_raw}" 2>&1 &
contact_pid=$!
sleep 1
# Contact can correctly prevent the requested terminal pose, so do not require
# work_a to converge.  The pass condition below is the measured CAD gripper /
# moving-jaw collision pair, not the trajectory return code.
set +e
ros2 run drone_arm_sim arm_preset_control \
  --preset work_a --duration 8 --wait --tolerance 0.12 \
  2>&1 | tee -a "${result_log}"
action_rc=${PIPESTATUS[0]}
set -e
wait "${contact_pid}" || true

python3 - "${contact_raw}" "${result_log}" "${action_rc}" <<'PY'
from pathlib import Path
import re
import sys

raw_path = Path(sys.argv[1])
result_path = Path(sys.argv[2])
action_rc = int(sys.argv[3])
text = raw_path.read_text(encoding="utf-8", errors="replace")
object_contacts = text.count("grasp_test_object::link::collision")
jaw_contacts = text.count("moving_jaw_link")
fixed_gripper_contacts = len(re.findall(r"my_drone::gripper_link", text))
depths = [float(value) for value in re.findall(r"depth:\s*([-+0-9.eE]+)", text)]
positive_depths = [value for value in depths if value > 0.0]
summary = (
    "GRASP_CONTACT_METRICS "
    f"object_contact_messages={object_contacts} "
    f"moving_jaw_mentions={jaw_contacts} "
    f"fixed_gripper_mentions={fixed_gripper_contacts} "
    f"max_positive_depth_m={max(positive_depths, default=0.0):.9g} "
    f"work_a_action_rc={action_rc}\n"
)
with result_path.open("a", encoding="utf-8") as stream:
    stream.write(summary)
    if object_contacts > 0 and (jaw_contacts > 0 or fixed_gripper_contacts > 0):
        stream.write("GRASP_CONTACT_PASS generic_fixture_only=true\n")
        passed = True
    else:
        stream.write("GRASP_CONTACT_FAIL no gripper/object collision pair\n")
        passed = False
print(summary, end="")
print("GRASP_CONTACT_PASS generic_fixture_only=true" if passed else "GRASP_CONTACT_FAIL")
raise SystemExit(0 if passed else 1)
PY
