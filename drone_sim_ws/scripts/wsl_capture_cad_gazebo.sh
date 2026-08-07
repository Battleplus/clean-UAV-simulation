#!/usr/bin/env bash
set -e
export GZ_PARTITION=my_drone_cad_display
output="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/analysis/cad_direct/gazebo_visible.png"
gz service \
  -s /gui/screenshot \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "data: \"${output}\""
echo "${output}"
