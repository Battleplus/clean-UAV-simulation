#!/usr/bin/env bash
set -e
export GZ_PARTITION=my_drone_cad_display
gz service \
  -s /gui/move_to \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'data: "my_drone"'
