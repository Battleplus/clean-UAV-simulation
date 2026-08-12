#!/usr/bin/env bash
# Start the 100 Hz read-only estimator without modifying Base 1 launch defaults.

set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="/tmp/my_drone_ros2_dds"
mkdir -p "${runtime_dir}"

set +u
source /opt/ros/jazzy/setup.bash
source /home/asus/ros2_px4_build_ws/install/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

if ! timeout 5 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
  echo "REFUSED: /joint_states is unavailable; start the Base 1 arm runtime first" >&2
  exit 2
fi

if [[ -f "${runtime_dir}/base1_estimator.pid" ]]; then
  prior="$(cat "${runtime_dir}/base1_estimator.pid")"
  kill -TERM "${prior}" 2>/dev/null || true
  sleep 1
fi

setsid ros2 run drone_arm_sim arm_coupling_monitor \
  --urdf "${workspace_dir}/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf" \
  --motion-reference "${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference.json" \
  --rate-hz 100 \
  --target-mass-kg 4.0 \
  --payload-mass-kg 0.0 \
  --feedforward-limit-m-s2 0.0 \
  --ros-args \
  -r __node:=base1_arm_coupling_estimator_100hz \
  -r /my_drone/arm_reaction_wrench_body:=/my_drone/base1_estimator/reaction_wrench_body \
  -r /my_drone/arm_gravity_shift_wrench_body:=/my_drone/base1_estimator/gravity_shift_wrench_body \
  -r /my_drone/arm_feedforward_acceleration_ned:=/my_drone/base1_estimator/candidate_acceleration_ned \
  -r /my_drone/arm_coupling_state:=/my_drone/base1_estimator/coupling_state \
  >"${runtime_dir}/base1_estimator.log" 2>&1 &
pid=$!
echo "${pid}" >"${runtime_dir}/base1_estimator.pid"

for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | \
      grep -qx /my_drone/base1_estimator/coupling_state; then
    echo "BASE1_READONLY_ESTIMATOR_READY pid=${pid} rate_hz=100 mass_kg=4.0"
    exit 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    cat "${runtime_dir}/base1_estimator.log" >&2 || true
    exit 1
  fi
  sleep 1
done

echo "Base 1 estimator overlay topic did not appear" >&2
kill -TERM "${pid}" 2>/dev/null || true
exit 1
