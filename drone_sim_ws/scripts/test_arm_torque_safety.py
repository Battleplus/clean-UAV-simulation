#!/usr/bin/env python3
"""Pure regression for the online arm reaction-torque LAND gate."""

from __future__ import annotations

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/px4_ros2_control"))

from px4_ros2_control.dds_wasd_control import reaction_torque_safety_triggered


def main() -> int:
    assert not reaction_torque_safety_triggered(np.array([0.1, 0.1, 0.1]), 0.1, 0.5, True)
    assert reaction_torque_safety_triggered(np.array([0.4, 0.4, 0.0]), 0.1, 0.5, True)
    assert not reaction_torque_safety_triggered(np.array([1.0, 0.0, 0.0]), 0.7, 0.5, True)
    assert not reaction_torque_safety_triggered(np.array([1.0, 0.0, 0.0]), 0.1, 0.5, False)
    assert not reaction_torque_safety_triggered(np.array([np.nan, 0.0, 0.0]), 0.1, 0.5, True)
    print("ARM_TORQUE_SAFETY_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
