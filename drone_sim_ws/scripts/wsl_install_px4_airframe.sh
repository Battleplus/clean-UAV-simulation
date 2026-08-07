#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_DIR:-/home/asus/PX4-Autopilot}"
airframes_dir="${px4_dir}/ROMFS/px4fmu_common/init.d-posix/airframes"
cmake_file="${airframes_dir}/CMakeLists.txt"
airframe_name="${1:-4015_gz_my_drone_octorotor}"

if [[ ! -f "${cmake_file}" ]]; then
  echo "PX4 airframe CMakeLists.txt not found under ${px4_dir}." >&2
  exit 1
fi

install -m 755 \
  "${workspace_dir}/px4/airframes/${airframe_name}" \
  "${airframes_dir}/${airframe_name}"

python3 - "${cmake_file}" "${airframe_name}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
name = sys.argv[2]
text = path.read_text(encoding="utf-8")
if f"\t{name}\n" not in text:
    anchor = "\t4014_gz_x500_mono_cam_down\n"
    if anchor not in text:
        raise SystemExit(f"Could not find the PX4 airframe insertion anchor in {path}")
    text = text.replace(anchor, anchor + f"\t{name}\n", 1)
    path.write_text(text, encoding="utf-8")
PY

venv_bin="${px4_dir}/.venv/bin"
if [[ -x "${venv_bin}/python" ]]; then
  export PATH="${venv_bin}:${PATH}"
fi
if [[ "${SKIP_PX4_BUILD:-0}" -ne 1 ]]; then
  make -C "${px4_dir}" px4_sitl_default
elif [[ -d "${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes" ]]; then
  install -m 755 \
    "${workspace_dir}/px4/airframes/${airframe_name}" \
    "${px4_dir}/build/px4_sitl_default/etc/init.d-posix/airframes/${airframe_name}"
fi

echo "Installed and built PX4 airframe ${airframe_name}"
