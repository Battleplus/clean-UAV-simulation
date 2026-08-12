import math

import numpy as np
import pytest

from px4_ros2_control.inner_loop_identification import (
    InnerLoopIdentification,
    euler_to_quaternion_wxyz,
    parse_args,
    quaternion_wxyz_to_rpy,
)


@pytest.mark.parametrize(
    "rpy",
    [
        (0.0, 0.0, 0.0),
        (math.radians(3.0), 0.0, math.radians(95.0)),
        (0.0, math.radians(-3.0), math.radians(-170.0)),
    ],
)
def test_quaternion_round_trip(rpy):
    recovered = quaternion_wxyz_to_rpy(euler_to_quaternion_wxyz(*rpy))
    error = recovered - np.asarray(rpy)
    error[2] = (error[2] + math.pi) % (2.0 * math.pi) - math.pi
    assert np.max(np.abs(error)) < 1.0e-9


def test_explicit_4kg_confirmation_is_not_implicit(tmp_path):
    options = parse_args(["--output", str(tmp_path / "report.json")])
    assert options.confirm_4kg_debug is False
    options = parse_args(
        ["--output", str(tmp_path / "report.json"), "--confirm-4kg-debug"]
    )
    assert options.confirm_4kg_debug is True


def test_v3_protocol_has_logged_baseline_and_measurable_plateaus():
    # PX4 mode switching must finish while a zero-rate setpoint is still being
    # streamed, otherwise a ULog can start at the non-zero step and invert the
    # baseline/return interpretation (the V2 roll-rate failure).
    assert InnerLoopIdentification.DIRECT_BASELINE_S >= 0.8

    # The analyzer requires a 0.30 s held settling band.  These plateaus leave
    # room for both a physical rise and that full settling observation.
    assert InnerLoopIdentification.ATTITUDE_STEP_S >= 1.2
    assert InnerLoopIdentification.RATE_ROLL_PITCH_STEP_S >= 1.0
    assert InnerLoopIdentification.RATE_YAW_STEP_S >= 1.0

    # 4 deg/s for 1.2 s keeps the commanded roll/pitch excursion below the
    # unchanged 8 degree direct-control safety gate.
    excursion_deg = 4.0 * InnerLoopIdentification.RATE_ROLL_PITCH_STEP_S
    assert excursion_deg < InnerLoopIdentification.MAX_TILT_DEG
