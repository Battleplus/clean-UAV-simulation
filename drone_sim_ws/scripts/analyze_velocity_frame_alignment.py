#!/usr/bin/env python3
"""Fit the horizontal Gazebo-ENU to PX4-local velocity frame rotation."""

import argparse
from pathlib import Path
import re

import numpy as np


STATE = re.compile(
    r"NED=\([^)]*\) vel=\(([-+0-9.e]+),([-+0-9.e]+),[-+0-9.e]+\).*"
    r"truth_vel_enu=\(([-+0-9.e]+),([-+0-9.e]+),[-+0-9.e]+\)"
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--minimum-speed", type=float, default=0.01)
    args = parser.parse_args()
    gazebo_enu = []
    px4_local = []
    for line in args.log.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = STATE.search(line)
        if not match:
            continue
        px4 = np.asarray([float(match.group(1)), float(match.group(2))])
        truth = np.asarray([float(match.group(3)), float(match.group(4))])
        if max(np.linalg.norm(px4), np.linalg.norm(truth)) < args.minimum_speed:
            continue
        gazebo_enu.append(truth)
        px4_local.append(px4)
    source = np.asarray(gazebo_enu)
    target = np.asarray(px4_local)
    if len(source) < 3:
        raise SystemExit("not enough moving samples")
    u_matrix, _, vt_matrix = np.linalg.svd(source.T @ target)
    row_rotation = u_matrix @ vt_matrix
    matrix = row_rotation.T
    estimate = source @ row_rotation
    print(f"samples={len(source)}")
    print("gazebo_enu_to_px4_local=")
    print(matrix)
    print(f"rotation_deg={np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0])):.6f}")
    print(f"rmse_m_s={np.sqrt(np.mean((estimate - target) ** 2)):.6f}")


if __name__ == "__main__":
    main()
