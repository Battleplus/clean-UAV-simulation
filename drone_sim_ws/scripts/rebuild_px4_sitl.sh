#!/usr/bin/env bash
set -o pipefail

cd /home/asus/PX4-Autopilot || exit 1
make px4_sitl_default -j4 2>&1 | tee /tmp/my_drone_px4_build.log
result=${PIPESTATUS[0]}
printf '%s\n' "$result" > /tmp/my_drone_px4_build.exit
exit "$result"
