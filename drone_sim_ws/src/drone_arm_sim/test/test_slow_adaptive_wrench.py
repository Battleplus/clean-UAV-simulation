import numpy as np
import pytest

from drone_arm_sim.slow_adaptive_wrench import SlowAdaptiveWrenchTrim


def advance(trim, residual, seconds, *, dt=0.1, **kwargs):
    """Advance using deterministic fixed steps and return the last sample."""
    value = trim.value.copy()
    for _ in range(round(seconds / dt)):
        value = trim.step(residual, dt, eligible=True, **kwargs)
    return value


def test_defaults_match_audited_base1_envelope():
    trim = SlowAdaptiveWrenchTrim()
    assert trim.time_constant_s == pytest.approx(15.0)
    assert trim.leak_time_constant_s == pytest.approx(60.0)
    assert trim.warmup_s == pytest.approx(3.0)
    assert trim.force_deadband_n == pytest.approx(0.02)
    assert trim.torque_deadband_nm == pytest.approx(0.005)
    assert trim.horizontal_force_limit_n == pytest.approx(0.06)
    assert trim.vertical_force_limit_n == pytest.approx(0.04)
    assert trim.torque_limit_nm == pytest.approx(0.02)
    assert trim.horizontal_force_rate_n_s == pytest.approx(0.005)
    assert trim.vertical_force_rate_n_s == pytest.approx(0.003)
    assert trim.torque_rate_nm_s == pytest.approx(0.002)


def test_warmup_requires_contiguous_quasi_static_eligibility():
    trim = SlowAdaptiveWrenchTrim(warmup_s=1.0)
    residual = np.asarray([0.04, 0.0, 0.0, 0.01, 0.0, 0.0])
    for _ in range(9):
        np.testing.assert_array_equal(
            trim.step(residual, 0.1, eligible=True), np.zeros(6)
        )
    assert trim.step(residual, 0.1, eligible=True)[0] > 0.0

    learned = trim.value.copy()
    np.testing.assert_array_equal(
        trim.step(residual, 0.1, eligible=True, quasi_static=False), learned
    )
    assert trim.eligible_duration_s == pytest.approx(0.0)
    for _ in range(9):
        np.testing.assert_array_equal(
            trim.step(residual, 0.1, eligible=True), learned
        )


def test_force_and_torque_deadbands_reject_px4_effort_noise():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    inside = np.asarray([0.012, 0.016, -0.02, 0.003, 0.004, 0.0])
    # XY norm is exactly 0.02 N; torque norm is exactly 0.005 Nm.
    value = advance(trim, inside, 120.0)
    np.testing.assert_allclose(value, np.zeros(6), atol=1.0e-15)

    outside = np.asarray([0.021, 0.0, -0.021, 0.0051, 0.0, 0.0])
    value = advance(trim, outside, 5.0)
    assert value[0] > 0.0
    assert value[2] < 0.0
    assert value[3] > 0.0


def test_constant_residual_converges_as_leaky_integral_and_zero_residual_leaks():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    # 0.03 N raw - 0.02 N deadband = 0.01 N effective.  The equilibrium of
    # residual/15 - trim/60 is 0.04 N, below the 0.06 N horizontal cap.
    value = advance(trim, np.asarray([0.03, 0, 0, 0, 0, 0]), 400.0)
    assert value[0] == pytest.approx(0.04, abs=6.0e-5)

    value_after_one_leak_time = advance(trim, np.zeros(6), 60.0)
    assert value_after_one_leak_time[0] == pytest.approx(
        0.04 * np.exp(-1.0), rel=0.01
    )


def test_group_rate_caps_apply_before_hard_limits():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    residual = np.asarray([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    first = trim.step(residual, 0.1, eligible=True)
    assert np.linalg.norm(first[:2]) == pytest.approx(0.005 * 0.1)
    assert first[2] == pytest.approx(0.003 * 0.1)
    assert np.linalg.norm(first[3:]) == pytest.approx(0.002 * 0.1)

    value = advance(trim, residual, 30.0)
    assert np.linalg.norm(value[:2]) <= 0.06 + 1.0e-12
    assert value[2] <= 0.04 + 1.0e-12
    assert np.linalg.norm(value[3:]) <= 0.02 + 1.0e-12
    assert np.linalg.norm(value[:2]) == pytest.approx(0.06)
    assert value[2] == pytest.approx(0.04)
    assert np.linalg.norm(value[3:]) == pytest.approx(0.02)


def test_allocation_limit_freezes_learning_without_back_calculation():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    residual = np.asarray([0.10, 0.0, 0.0, 0.03, 0.0, 0.0])
    advance(trim, residual, 10.0)
    learned = trim.value.copy()
    for _ in range(100):
        frozen = trim.step(
            -residual, 0.1, eligible=True, allocation_limited=True
        )
    np.testing.assert_array_equal(frozen, learned)


def test_allocation_limit_back_calculates_to_actual_delivery_and_does_not_wind_up():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    residual = np.asarray([0.10, 0.0, 0.0, 0.03, 0.0, 0.0])
    advance(trim, residual, 10.0)
    delivered = np.asarray([0.006, -0.002, 0.003, 0.001, 0.0, -0.001])
    value = trim.step(
        residual,
        0.1,
        eligible=True,
        allocation_limited=True,
        delivered_wrench_frd=delivered,
    )
    np.testing.assert_allclose(value, delivered)
    for _ in range(100):
        value = trim.step(
            residual,
            0.1,
            eligible=True,
            allocation_limited=True,
        )
    np.testing.assert_allclose(value, delivered)


def test_trim_persists_across_actions_until_explicit_reset():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    residual = np.asarray([0.06, 0.0, 0.0, 0.01, 0.0, 0.0])
    advance(trim, residual, 20.0)
    learned = trim.value.copy()
    assert np.linalg.norm(learned) > 0.0

    # An action boundary, stale sample, or invalid residual must not silently
    # erase calibration.  The supervisor owns the explicit reset decision.
    for _ in range(100):
        np.testing.assert_array_equal(
            trim.step(residual, 0.1, eligible=False), learned
        )
    np.testing.assert_array_equal(
        trim.step(np.full(6, np.nan), 0.1, eligible=True), learned
    )
    trim.reset()
    np.testing.assert_array_equal(trim.value, np.zeros(6))
    assert trim.eligible_duration_s == pytest.approx(0.0)


def test_legacy_decay_name_maps_to_leak_without_ineligible_decay():
    trim = SlowAdaptiveWrenchTrim(
        warmup_s=0.0,
        decay_time_constant_s=30.0,
    )
    assert trim.leak_time_constant_s == pytest.approx(30.0)
    trim.value[:] = 0.01
    value = trim.step(np.zeros(6), 0.1, eligible=False)
    np.testing.assert_array_equal(value, np.full(6, 0.01))


def test_invalid_dt_and_invalid_delivery_fail_closed():
    trim = SlowAdaptiveWrenchTrim(warmup_s=0.0)
    trim.value[:] = 0.01
    learned = trim.value.copy()
    value = trim.step(
        np.ones(6),
        float("nan"),
        eligible=True,
        delivered_wrench_frd=np.full(6, np.nan),
    )
    np.testing.assert_array_equal(value, learned)
    np.testing.assert_array_equal(
        trim.step(None, 0.1, eligible=True), learned
    )
    np.testing.assert_array_equal(trim.back_calculate("not-a-wrench"), learned)
