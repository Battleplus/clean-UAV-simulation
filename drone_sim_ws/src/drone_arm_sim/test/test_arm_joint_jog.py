import pytest

from drone_arm_sim.arm_joint_jog import bounded_jog_target
from drone_arm_sim.arm_preset_control import JOINT_NAMES


def reference():
    return {
        "joints": [
            {"name": name, "lower_rad": -1.0, "upper_rad": 1.0}
            for name in JOINT_NAMES
        ]
    }


def test_bounded_jog_changes_only_requested_joint():
    target = bounded_jog_target(
        [0.0] * len(JOINT_NAMES), "elbow_flex", 0.05, reference()
    )
    assert target == [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]


def test_bounded_jog_accumulates_from_measured_current_pose():
    current = [0.2, -0.1, 0.3, 0.0, -0.5, 0.4]
    target = bounded_jog_target(current, "gripper", -0.1, reference())
    assert target[:-1] == current[:-1]
    assert target[-1] == pytest.approx(0.3)


def test_bounded_jog_rejects_limit_crossing_instead_of_clipping():
    with pytest.raises(ValueError, match="outside"):
        bounded_jog_target([0.99] + [0.0] * 5, "shoulder_pan", 0.05, reference())


def test_bounded_jog_rejects_incomplete_measurement():
    with pytest.raises(ValueError, match="all SO101 joints"):
        bounded_jog_target([0.0], "shoulder_pan", 0.05, reference())
