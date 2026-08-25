#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
runtime_dir="/tmp/my_drone_ros2_dds"
cpu_role_runner="${workspace_dir}/scripts/run_with_cpu_role.sh"
mkdir -p "${runtime_dir}"
model_settle_s="${MODEL_SETTLE_S:-8}"
px4_ready_settle_s="${PX4_READY_SETTLE_S:-5}"

# Keep the PX4 SITL build's generated airframe in sync with the authoritative
# project copy.  PX4 executes the file under build/.../etc at runtime; merely
# editing drone_sim_ws/px4/airframes would otherwise leave an older parameter
# set active until a full PX4 rebuild.
project_airframe="${PROJECT_AIRFRAME_FILE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../px4/airframes" && pwd)/4026_gz_my_drone_octorotor_7p735}"
px4_build_airframe="${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/4026_gz_my_drone_octorotor_7p735"
if [[ -f "${project_airframe}" && -d "$(dirname "${px4_build_airframe}")" ]]; then
  cp "${project_airframe}" "${px4_build_airframe}"
fi

# A second launch would create another direct-wrench node subscribing to the
# same actuator topic and apply thrust twice to one Gazebo entity.
if [[ "${CLEAN_STALE_RUNTIME:-1}" == "1" ]]; then
  pkill -x gazebo_direct_m 2>/dev/null || true
  # Optional Base 1 overlays outlive the launch parent by design.  Remove
  # their installed entry points explicitly so a clean restart cannot retain
  # a second motor-command publisher or a stale compensation state.
  pkill -f '/[b]ase1_wrench_reallocator' 2>/dev/null || true
  pkill -f '/[a]rm_coupling_monitor .*__node:=base1_arm_coupling_estimator_100hz' 2>/dev/null || true
  pkill -f '/[c]artesian_arm_velocity_control' 2>/dev/null || true
  pkill -x gazebo_sensor_d 2>/dev/null || true
  pkill -x parameter_bridg 2>/dev/null || true
  pkill -x robot_state_pub 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  pkill -x px4 2>/dev/null || true
  pkill -x MicroXRCEAgent 2>/dev/null || true
  # Gazebo Sim 8 is launched by a Ruby wrapper.  Depending on how ros2 launch
  # was interrupted, the wrapper may ignore SIGTERM or be re-parented, so its
  # process name alone is not a reliable cleanup key.  Match the actual server
  # command line; the bracketed pattern deliberately does not match this shell.
  pkill -TERM -f '[g]z sim -s -r' 2>/dev/null || true
  pkill -x ruby 2>/dev/null || true
  # ROS launch and the DDS keyboard controller both have process name ros2.
  pkill -x ros2 2>/dev/null || true
  pkill -f '/arm_coupling_monitor' 2>/dev/null || true
  # A PTY test may leave the installed Python entry point behind if the
  # parent shell is interrupted; never allow two DDS Offboard publishers.
  pkill -f '/px4_ros2_control/dds_wasd_control' 2>/dev/null || true
  pkill -f '/px4_ros2_control/direct_xy_guardian' 2>/dev/null || true
  # The arm keyboard is a plain bash loop (not a ros2 process), so a prior
  # visible terminal can otherwise survive a clean backend restart and leave
  # two operator consoles on screen.
  pkill -f '/run_ros2_arm_keyboard.sh' 2>/dev/null || true
  sleep 1
  # Do not let an unresponsive old physics server keep publishing /clock and
  # /stats into the new run.  This is intentionally scoped to headless
  # `gz sim -s -r` servers rather than all Gazebo processes/GUI windows.
  pkill -KILL -f '[g]z sim -s -r' 2>/dev/null || true
  # PID files refer to `setsid` wrapper processes and must never survive a
  # clean restart.  A recycled numeric PID could otherwise make a later
  # overlay stop an unrelated process.
  rm -f \
    "${runtime_dir}/agent.pid" \
    "${runtime_dir}/gazebo.pid" \
    "${runtime_dir}/px4.pid" \
    "${runtime_dir}/base1_estimator.pid" \
    "${runtime_dir}/base1_reallocator.pid" \
    "${runtime_dir}/direct_xy_guardian.pid" \
    "${runtime_dir}/base1_overlay_motor.pid" \
    "${runtime_dir}/cartesian_velocity.pid"
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
export LD_LIBRARY_PATH="/home/asus/ros2_px4_build_ws/install/px4_msgs/lib:/home/asus/.local/lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

agent_log="${runtime_dir}/agent.log"
gazebo_log="${runtime_dir}/gazebo.log"
px4_log="${runtime_dir}/px4.log"
arm_init_log="${runtime_dir}/arm_init.log"
settle_log="${runtime_dir}/settle.log"
px4_stability_log="${runtime_dir}/px4_stability.log"
: >"${agent_log}"
: >"${gazebo_log}"
: >"${px4_log}"
: >"${arm_init_log}"
: >"${settle_log}"
: >"${px4_stability_log}"

setsid "${cpu_role_runner}" bulk \
  /home/asus/.local/bin/MicroXRCEAgent udp4 -p 8888 -v 4 \
  >"${agent_log}" 2>&1 &
echo $! >"${runtime_dir}/agent.pid"

default_robot_file="${workspace_dir}/install/drone_arm_sim/share/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
default_config_file="${workspace_dir}/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json"
export MY_DRONE_URDF="${ROBOT_FILE:-${default_robot_file}}"
setsid "${cpu_role_runner}" bulk \
  ros2 launch drone_arm_sim cad_direct_thrust.launch.py \
  headless:="${HEADLESS:-false}" enable_controller:=false \
  enable_arm_control:="${ENABLE_ARM_CONTROL:-false}" \
  gz_seed:="${GZ_RANDOM_SEED:-4027}" \
  spawn_z:="${SPAWN_Z:-0.289}" \
  reaction_moment_ratio_m:="${REACTION_MOMENT_RATIO_M:--1}" \
  wind_enu_x:="${WIND_ENU_X:-nan}" wind_enu_y:="${WIND_ENU_Y:-nan}" \
  wind_enu_z:="${WIND_ENU_Z:-nan}" \
  battery_dynamics_enabled:="${BATTERY_DYNAMICS_ENABLED:-false}" \
  enable_sensor_delay:="${ENABLE_SENSOR_DELAY:-true}" \
  imu_delay_ms:="${IMU_DELAY_MS:-0}" \
  mag_delay_ms:="${MAG_DELAY_MS:-10}" \
  baro_delay_ms:="${BARO_DELAY_MS:-20}" \
  navsat_delay_ms:="${NAVSAT_DELAY_MS:-50}" \
  battery_internal_resistance_ohm:="${BATTERY_INTERNAL_RESISTANCE_OHM:-nan}" \
  battery_capacity_ah:="${BATTERY_CAPACITY_AH:-nan}" \
  battery_full_voltage_v:="${BATTERY_FULL_VOLTAGE_V:-nan}" \
  battery_empty_voltage_v:="${BATTERY_EMPTY_VOLTAGE_V:-nan}" \
  battery_minimum_loaded_voltage_v:="${BATTERY_MINIMUM_LOADED_VOLTAGE_V:-nan}" \
  battery_thrust_voltage_exponent:="${BATTERY_THRUST_VOLTAGE_EXPONENT:-nan}" \
  arm_torque_feedforward_enabled:="${ARM_TORQUE_FEEDFORWARD_ENABLED:-false}" \
  arm_reaction_torque_feedforward_gain:="${ARM_REACTION_TORQUE_FEEDFORWARD_GAIN:-1.0}" \
  arm_torque_feedforward_max_delta_n:="${ARM_TORQUE_FEEDFORWARD_MAX_DELTA_N:-2.0}" \
  arm_static_com_feedforward_gain:="${ARM_STATIC_COM_FEEDFORWARD_GAIN:-0.0}" \
  arm_static_com_feedforward_time_constant_s:="${ARM_STATIC_COM_FEEDFORWARD_TIME_CONSTANT_S:-5.0}" \
  arm_disturbance_observer_enabled:="${ARM_DISTURBANCE_OBSERVER_ENABLED:-false}" \
  arm_disturbance_observer_gain:="${ARM_DISTURBANCE_OBSERVER_GAIN:-0.5}" \
  arm_disturbance_observer_max_torque_nm:="${ARM_DISTURBANCE_OBSERVER_MAX_TORQUE_NM:-0.08}" \
  arm_disturbance_observer_max_delta_n:="${ARM_DISTURBANCE_OBSERVER_MAX_DELTA_N:-1.0}" \
  arm_coupling_target_mass_kg:="${ARM_COUPLING_TARGET_MASS_KG:-7.735}" \
  arm_payload_mass_kg:="${ARM_PAYLOAD_MASS_KG:-0.0}" \
  config_file:="${CONFIG_FILE:-${default_config_file}}" \
  >"${gazebo_log}" 2>&1 &
echo $! >"${runtime_dir}/gazebo.pid"

for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/model/my_drone/link/base_link/sensor/imu_sensor/imu'; then
    break
  fi
  sleep 1
done
# The retracted CAD gripper rests on the ground and can bounce while the
# sensors first come online.  Let the rigid body settle before PX4 chooses its
# local-position origin; otherwise the first takeoff target contains the fall
# distance and the safety gate correctly aborts it.
echo "Waiting ${model_settle_s}s for the spawned CAD model to settle" >>"${gazebo_log}"
sleep "${model_settle_s}"
if ! python3 "${workspace_dir}/scripts/wait_model_settled.py" \
  --timeout "${MODEL_SETTLE_TIMEOUT_S:-45}" \
  --hold "${MODEL_SETTLE_HOLD_S:-2}" \
  --linear-limit "${MODEL_SETTLE_LINEAR_LIMIT_M_S:-0.08}" \
  --angular-limit "${MODEL_SETTLE_ANGULAR_LIMIT_RAD_S:-0.08}" \
  >"${settle_log}" 2>&1; then
  echo "Gazebo CAD model did not settle; refusing to start PX4" >&2
  cat "${settle_log}" >&2 || true
  tail -n 80 "${gazebo_log}" >&2 || true
  if [[ -f "${runtime_dir}/gazebo.pid" ]]; then
    kill -- "-$(cat "${runtime_dir}/gazebo.pid")" 2>/dev/null || true
  fi
  exit 1
fi

cd "${px4_dir}"
px4_command=(./build/px4_sitl_default/bin/px4 -d)
if [[ "${PX4_FRESH_WORKDIR:-0}" == "1" ]]; then
  px4_fresh_workdir="$(mktemp -d /tmp/my_drone_px4_work.XXXXXX)"
  printf '%s\n' "${px4_fresh_workdir}" >"${runtime_dir}/px4_workdir.path"
  px4_command+=(
    -w "${px4_fresh_workdir}"
    "${px4_dir}/build/px4_sitl_default/etc"
  )
fi
setsid "${cpu_role_runner}" bulk \
  env PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=flight_world \
  PX4_GZ_MODEL_NAME=my_drone PX4_SYS_AUTOSTART="${AIRFRAME_ID:-4026}" \
  "${px4_command[@]}" >"${px4_log}" 2>&1 &
echo $! >"${runtime_dir}/px4.pid"

for _ in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -Eq '^/fmu/out/vehicle_status(_v[0-9]+)?$'; then
    if [[ "${ENABLE_ARM_CONTROL:-false}" == "true" ]]; then
      # Position interfaces do not hold a gravity-loaded arm until the first
      # trajectory is received.  Freeze the documented CAD retracted pose
      # before any flight controller is allowed to arm.
      # On a slow GUI start the joint-state broadcaster can time out while the
      # arm trajectory controller is still activating.  In that case Gazebo,
      # PX4 and the raw joint bridge are healthy, but /joint_states is absent
      # and the preset command would terminate this launcher with status 1.
      # Retry the already loaded broadcaster through controller_manager, then
      # require one real /joint_states sample before sending any trajectory.
      arm_joint_state_topic="${ARM_JOINT_STATE_TOPIC:-/joint_states}"
      arm_joint_sample_timeout_s="${ARM_INIT_SAMPLE_TIMEOUT_S:-8}"
      arm_joint_sample_args=(--joint-samples 3 --timeout "${arm_joint_sample_timeout_s}")
      if [[ "${arm_joint_state_topic}" != "/joint_states" ]]; then
        # The persistent sample gate intentionally uses the authoritative
        # /joint_states stream.  Keep the generic fallback for an explicitly
        # overridden diagnostic topic.
        arm_joint_sample_args=()
      fi
      if { [[ "${#arm_joint_sample_args[@]}" -gt 0 ]] && \
          python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
            "${arm_joint_sample_args[@]}" >/dev/null 2>&1; } || \
          { [[ "${#arm_joint_sample_args[@]}" -eq 0 ]] && \
          timeout "${arm_joint_sample_timeout_s}" ros2 topic echo --once "${arm_joint_state_topic}" \
            >/dev/null 2>&1; }; then
        arm_joint_state_ready=true
      else
        echo "ARM_INIT_RETRY activating joint_state_broadcaster" \
          >>"${arm_init_log}"
        timeout 20 ros2 service call \
          /controller_manager/switch_controller \
          controller_manager_msgs/srv/SwitchController \
          "{activate_controllers: [joint_state_broadcaster], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 15, nanosec: 0}}" \
          >>"${arm_init_log}" 2>&1 || true
        arm_joint_state_ready=false
        if [[ "${#arm_joint_sample_args[@]}" -gt 0 ]]; then
          if python3 "${workspace_dir}/scripts/wait_base1_ros_samples.py" \
              --joint-samples 3 --timeout "${ARM_INIT_RETRY_TIMEOUT_S:-30}" \
              >/dev/null 2>&1; then
            arm_joint_state_ready=true
          fi
        elif timeout "${ARM_INIT_RETRY_TIMEOUT_S:-30}" ros2 topic echo --once "${arm_joint_state_topic}" \
            >/dev/null 2>&1; then
          arm_joint_state_ready=true
        fi
      fi
      if [[ "${arm_joint_state_ready}" != "true" ]]; then
        echo "ARM_INIT_FAIL joint state unavailable: ${arm_joint_state_topic}" \
          >"${arm_init_log}"
        cat "${arm_init_log}" >&2
        exit 1
      fi
      # This is a disarmed ground-initialisation command executed before the
      # DDS flight controller and direct-XY reallocator exist.  It must not
      # wait for the airborne ownership handshake inherited from a candidate
      # launcher environment.
      ARM_DIRECT_XY_OWNERSHIP=false \
      ros2 run drone_arm_sim arm_preset_control \
        --preset retracted --duration 8 --wait --tolerance 0.08 \
        >>"${arm_init_log}" 2>&1
      # Reaching a position tolerance is not enough for base-airframe
      # identification: a still-settling arm would contaminate the measured
      # body-rate and attitude response.  Require every SO101 joint to remain
      # near the retracted target and below the velocity limit continuously.
      python3 "${workspace_dir}/scripts/wait_arm_static.py" \
        --reference "${SO101_MOTION_REFERENCE:-${workspace_dir}/src/drone_arm_sim/config/so101_motion_reference.json}" \
        --preset retracted \
        --topic "${arm_joint_state_topic}" \
        --position-tolerance "${ARM_STATIC_POSITION_TOLERANCE_RAD:-0.08}" \
        --velocity-limit "${ARM_STATIC_VELOCITY_LIMIT_RAD_S:-0.03}" \
        --hold "${ARM_STATIC_HOLD_S:-2}" \
        --timeout "${ARM_STATIC_TIMEOUT_S:-30}" \
        >>"${arm_init_log}" 2>&1
    fi
    # DDS topic discovery is not evidence that the estimator is ready.  In
    # particular, the formal 7.735 kg setup has occasionally reported a false
    # 0.5-0.6 m/s vertical velocity for several seconds after Gazebo itself is
    # already stationary.  Never let an automated test arm from that state.
    sleep "${px4_ready_settle_s}"
    if ! python3 "${workspace_dir}/scripts/wait_px4_stable.py" \
      --horizontal "${PX4_READY_HORIZONTAL_LIMIT_M_S:-0.10}" \
      --vertical "${PX4_READY_VERTICAL_LIMIT_M_S:-0.08}" \
      --hold "${PX4_READY_STABLE_HOLD_S:-5}" \
      --timeout "${PX4_READY_STABLE_TIMEOUT_S:-90}" \
      >"${px4_stability_log}" 2>&1; then
      echo "PX4 local-position estimator did not stabilize; refusing to arm" >&2
      cat "${px4_stability_log}" >&2 || true
      exit 1
    fi
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
