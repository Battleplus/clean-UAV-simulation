from __future__ import annotations

import math

import pytest

from wait_arm_static import evaluate_joint_sample


TARGET = {"joint_a": 0.0, "joint_b": -1.5}


def test_joint_gate_accepts_reordered_complete_static_sample():
    complete, error, speed = evaluate_joint_sample(
        ["joint_b", "joint_a"], [-1.49, 0.01], [0.002, -0.001], TARGET
    )
    assert complete is True
    assert error == pytest.approx(0.01)
    assert speed == pytest.approx(0.002)


def test_joint_gate_rejects_missing_velocity_or_joint():
    complete, error, speed = evaluate_joint_sample(
        ["joint_a"], [0.0], [0.0], TARGET
    )
    assert complete is False
    assert math.isinf(error)
    assert math.isinf(speed)

    complete, _, _ = evaluate_joint_sample(
        ["joint_a", "joint_b"], [0.0, -1.5], [0.0], TARGET
    )
    assert complete is False


def test_joint_gate_rejects_nonfinite_measurement():
    complete, error, speed = evaluate_joint_sample(
        ["joint_a", "joint_b"], [0.0, float("nan")], [0.0, 0.0], TARGET
    )
    assert complete is False
    assert math.isinf(error)
    assert math.isinf(speed)
