#!/usr/bin/env bash
set -o pipefail

cd /mnt/e/清洁无人机/drone_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

log=analysis/cartesian_velocity_joint_4kg_p220.log
status=analysis/cartesian_velocity_joint_4kg_p220.status
rm -f "$status"

ARM_FLIGHT_PROFILE=cartesian_velocity_4kg \
PX4_POSITION_KP=2.20 \
PX4_POSITION_KD=0.35 \
PX4_MOMENT_CONSTANT=0.001 \
python3 scripts/test_ros2_dds_arm_flight_pty.py --timeout 210 \
  >"$log" 2>&1
result=$?
printf '%s\n' "$result" >"$status"
exit "$result"
