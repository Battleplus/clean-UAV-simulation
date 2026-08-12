import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from test_ros2_dds_velocity_wasd_pty import (  # noqa: E402
    _horizontal_phase_response_metrics,
    _scalar_phase_response_metrics,
)


def test_scalar_phase_metrics_report_rise_settling_overshoot_and_steady_error():
    samples = [
        {"elapsed_s": 0.0, "actual": 0.0, "command": 0.0},
        {"elapsed_s": 1.0, "actual": 8.0, "command": 10.0},
        {"elapsed_s": 2.0, "actual": 15.5, "command": 15.0},
        {"elapsed_s": 3.0, "actual": 15.0, "command": 15.0},
        {"elapsed_s": 4.0, "actual": 14.9, "command": 15.0},
    ]
    metrics = _scalar_phase_response_metrics(
        samples, 15.0, "actual", "command", "deg_s", 1.5
    )
    assert metrics["rise_time_s"] == pytest.approx(2.0)
    assert metrics["overshoot_fraction"] == pytest.approx(0.5 / 15.0)
    assert metrics["settling_time_s"] == pytest.approx(2.0)
    assert metrics["steady_actual_deg_s"] == pytest.approx(14.95)
    assert metrics["steady_error_deg_s"] == pytest.approx(0.05)


def test_horizontal_reversal_projects_onto_latest_command_direction():
    samples = [
        {
            "elapsed_s": 0.0,
            "actual_vx": 0.4,
            "actual_vy": 0.0,
            "command_vx": 0.3,
            "command_vy": 0.0,
        },
        {
            "elapsed_s": 1.0,
            "actual_vx": 0.1,
            "actual_vy": 0.0,
            "command_vx": -0.1,
            "command_vy": 0.0,
        },
        {
            "elapsed_s": 2.0,
            "actual_vx": -0.37,
            "actual_vy": 0.0,
            "command_vx": -0.4,
            "command_vy": 0.0,
        },
        {
            "elapsed_s": 3.0,
            "actual_vx": -0.4,
            "actual_vy": 0.0,
            "command_vx": -0.4,
            "command_vy": 0.0,
        },
    ]
    metrics = _horizontal_phase_response_metrics(samples, 0.4)
    assert metrics["projection_direction_ned"] == pytest.approx([-1.0, 0.0])
    assert metrics["rise_time_s"] == pytest.approx(2.0)
    assert metrics["steady_actual_m_s"] == pytest.approx(0.385)
    assert metrics["steady_error_m_s"] == pytest.approx(0.015)
