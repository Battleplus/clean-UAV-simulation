from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT = Path(__file__).with_name("analyze_control_loops_ulog.py")
SPEC = importlib.util.spec_from_file_location("analyze_control_loops_ulog", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_clean_step_reports_rise_settling_and_zero_steady_error():
    time_s = np.arange(0.0, 5.0, 0.02)
    target = np.where(time_s >= 1.0, 2.0, 0.0)
    actual = np.where(time_s >= 1.0, 2.0 * (1.0 - np.exp(-(time_s - 1.0) / 0.25)), 0.0)
    metrics = MODULE.response_metrics(
        time_s,
        target,
        actual,
        MODULE.ResponseLimits(amplitude_floor=0.1),
    )
    assert metrics["step_evaluable"] is True
    assert 0.4 < metrics["rise_time_s"] < 0.8
    assert metrics["overshoot_fraction"] == pytest.approx(0.0)
    assert metrics["settling_time_s"] is not None
    assert abs(metrics["steady_error"]) < 1.0e-4


def test_overshoot_is_measured_against_dominant_signed_step():
    time_s = np.arange(0.0, 4.0, 0.02)
    target = np.where(time_s >= 0.5, -1.0, 0.0)
    actual = np.zeros_like(time_s)
    moving = time_s >= 0.5
    actual[moving] = -1.2 * (1.0 - np.exp(-(time_s[moving] - 0.5) / 0.15))
    metrics = MODULE.response_metrics(
        time_s,
        target,
        actual,
        MODULE.ResponseLimits(amplitude_floor=0.1),
    )
    assert metrics["step_evaluable"] is True
    assert metrics["overshoot_fraction"] == pytest.approx(0.2, abs=0.01)


def test_dynamic_command_without_plateau_is_not_called_a_step():
    time_s = np.arange(0.0, 4.0, 0.02)
    target = np.sin(2.0 * np.pi * time_s)
    actual = 0.9 * np.sin(2.0 * np.pi * (time_s - 0.05))
    metrics = MODULE.response_metrics(
        time_s,
        target,
        actual,
        MODULE.ResponseLimits(amplitude_floor=0.1, plateau_min_s=0.4),
    )
    assert metrics["step_evaluable"] is False
    assert metrics["reason"] == "dominant_command_has_no_stable_plateau"


def test_small_command_is_explicitly_not_evaluable():
    time_s = np.arange(0.0, 2.0, 0.02)
    target = np.full_like(time_s, 0.01)
    actual = np.full_like(time_s, 0.008)
    metrics = MODULE.response_metrics(
        time_s,
        target,
        actual,
        MODULE.ResponseLimits(amplitude_floor=0.05),
    )
    assert metrics["step_evaluable"] is False
    assert metrics["reason"] == "command_amplitude_below_floor"


def test_segmented_report_selects_matching_axis_step_not_recovery_gap():
    first_t = np.arange(0.0, 2.0, 0.02)
    second_t = np.arange(5.0, 7.0, 0.02)
    time_s = np.concatenate((first_t, second_t))
    target = np.zeros((len(time_s), 2))
    actual = np.zeros_like(target)
    target[first_t.size // 4 : first_t.size, 0] = 0.2
    actual[first_t.size // 4 : first_t.size, 0] = 0.19
    offset = len(first_t)
    target[offset + second_t.size // 4 :, 1] = -0.3
    actual[offset + second_t.size // 4 :, 1] = -0.29
    report = MODULE._segmented_axis_report(
        time_s,
        target,
        actual,
        ("x", "y"),
        MODULE.ResponseLimits(amplitude_floor=0.05),
    )
    assert report["x"]["target_value"] == pytest.approx(0.2)
    assert report["x"]["segment_end_s"] < 3.0
    assert report["y"]["target_value"] == pytest.approx(-0.3)
    assert report["y"]["segment_start_s"] > 4.0
