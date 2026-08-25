"""Focused fail-closed tests for arm coupling motion-state publication."""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import drone_arm_sim.arm_coupling_monitor as monitor_module
from drone_arm_sim.arm_coupling_monitor import (
    ArmCouplingMonitor,
    desired_reference_gate,
    evaluate_coupling_sources,
    filtered_joint_acceleration_step,
    joint_motion_metrics,
    validate_controller_reference,
    validate_joint_state_sample,
)
from drone_arm_sim.coupled_dynamics import JOINT_NAMES


def _message(positions, velocities=(), stamp_s=1.0):
    seconds = int(stamp_s)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=seconds, nanosec=int(round((stamp_s - seconds) * 1.0e9))
            )
        ),
        name=list(JOINT_NAMES),
        position=list(positions),
        velocity=list(velocities),
    )


def _controller_message(
    names,
    positions,
    velocities,
    accelerations,
    stamp_s=1.0,
):
    seconds = int(stamp_s)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=seconds, nanosec=int(round((stamp_s - seconds) * 1.0e9))
            )
        ),
        joint_names=list(names),
        reference=SimpleNamespace(
            positions=list(positions),
            velocities=list(velocities),
            accelerations=list(accelerations),
        ),
    )


def _bare_monitor():
    node = object.__new__(ArmCouplingMonitor)
    node.positions = {}
    node.velocities = {}
    node.accelerations = {}
    node.previous_velocities = {}
    node.previous_joint_time = None
    node.last_joint_source_stamp_s = None
    node.last_joint_receipt_monotonic_s = None
    node.last_joint_sample_valid = False
    node.last_joint_invalid_reason = "no_joint_state"
    node.acceleration_filter_time_constant_s = 0.20
    node.maximum_joint_acceleration_rad_s2 = 4.0
    node.raw_joint_acceleration_peak_rad_s2 = 0.0
    node.dynamic_reaction_enabled = False
    node.reference_timeout_s = 0.10
    node.reference_tracking_error_rad = 0.20
    node.reference_positions = {}
    node.reference_velocities = {}
    node.reference_accelerations = {}
    node.last_reference_source_stamp_s = None
    node.last_reference_receipt_monotonic_s = None
    node.last_reference_sample_valid = False
    node.last_reference_invalid_reason = "no_controller_reference"
    return node


def test_complete_finite_sample_accepts_optional_velocity_vector():
    positions, velocities, reason = validate_joint_state_sample(
        JOINT_NAMES, [0.1] * len(JOINT_NAMES), []
    )
    assert reason is None
    assert velocities is None
    assert positions == {name: 0.1 for name in JOINT_NAMES}

    positions, velocities, reason = validate_joint_state_sample(
        JOINT_NAMES, [0.1] * len(JOINT_NAMES), [0.2] * len(JOINT_NAMES)
    )
    assert reason is None
    assert velocities == {name: 0.2 for name in JOINT_NAMES}


def test_incomplete_duplicate_and_nonfinite_samples_fail_closed():
    _, _, reason = validate_joint_state_sample(
        JOINT_NAMES[:-1], [0.0] * (len(JOINT_NAMES) - 1), []
    )
    assert reason.startswith("missing_required_joints:")

    duplicate_names = list(JOINT_NAMES)
    duplicate_names[-1] = duplicate_names[0]
    _, _, reason = validate_joint_state_sample(
        duplicate_names, [0.0] * len(duplicate_names), []
    )
    assert reason == "duplicate_joint_name"

    bad_positions = [0.0] * len(JOINT_NAMES)
    bad_positions[2] = float("nan")
    _, _, reason = validate_joint_state_sample(JOINT_NAMES, bad_positions, [])
    assert reason == "non_finite_joint_position"

    bad_velocities = [0.0] * len(JOINT_NAMES)
    bad_velocities[4] = float("inf")
    _, _, reason = validate_joint_state_sample(
        JOINT_NAMES, [0.0] * len(JOINT_NAMES), bad_velocities
    )
    assert reason == "non_finite_joint_velocity"


def test_controller_reference_is_complete_finite_and_reordered_by_name():
    names = list(reversed(JOINT_NAMES)) + ["unrelated_joint"]
    positions = [float(index) for index in range(len(names))]
    velocities = [10.0 + value for value in positions]
    accelerations = [20.0 + value for value in positions]
    q, qd, qdd, reason = validate_controller_reference(
        names, positions, velocities, accelerations
    )
    assert reason is None
    for name in JOINT_NAMES:
        source_index = names.index(name)
        assert q[name] == positions[source_index]
        assert qd[name] == velocities[source_index]
        assert qdd[name] == accelerations[source_index]


def test_controller_reference_rejects_partial_or_nonfinite_derivatives():
    _, _, _, reason = validate_controller_reference(
        JOINT_NAMES,
        [0.0] * len(JOINT_NAMES),
        [0.0] * len(JOINT_NAMES),
        [],
    )
    assert reason == "reference_acceleration_size_mismatch"

    bad_accelerations = [0.0] * len(JOINT_NAMES)
    bad_accelerations[-1] = float("nan")
    _, _, _, reason = validate_controller_reference(
        JOINT_NAMES,
        [0.0] * len(JOINT_NAMES),
        [0.0] * len(JOINT_NAMES),
        bad_accelerations,
    )
    assert reason == "reference_non_finite_joint_acceleration"


def test_controller_reference_callback_rejects_out_of_order_without_reuse():
    node = _bare_monitor()
    complete = [0.1] * len(JOINT_NAMES)
    with patch.object(monitor_module.time, "monotonic", return_value=5.0):
        node.on_controller_state(
            _controller_message(JOINT_NAMES, complete, complete, complete, 2.0)
        )
    accepted = dict(node.reference_positions)
    assert node.last_reference_sample_valid

    with patch.object(monitor_module.time, "monotonic", return_value=5.01):
        node.on_controller_state(
            _controller_message(
                JOINT_NAMES,
                [0.2] * len(JOINT_NAMES),
                complete,
                complete,
                1.99,
            )
        )
    assert not node.last_reference_sample_valid
    assert node.last_reference_invalid_reason == "reference_out_of_order"
    assert node.reference_positions == accepted


def test_dynamic_reference_gate_defaults_off_and_fails_closed():
    actual = {name: 0.0 for name in JOINT_NAMES}
    reference = dict(actual)
    active, reason, error = desired_reference_gate(
        False, True, None, 0.01, 0.10, actual, reference, 0.20
    )
    assert not active
    assert reason == "dynamic_reaction_disabled"
    assert error is None

    active, reason, error = desired_reference_gate(
        True, True, None, 0.11, 0.10, actual, reference, 0.20
    )
    assert not active
    assert reason == "controller_reference_stale"
    assert error is None

    active, reason, error = desired_reference_gate(
        True,
        False,
        "reference_out_of_order",
        0.01,
        0.10,
        actual,
        reference,
        0.20,
    )
    assert not active
    assert reason == "reference_out_of_order"
    assert error is None


def test_dynamic_reference_tracking_gate_enforces_limit():
    actual = {name: 0.0 for name in JOINT_NAMES}
    reference = dict(actual)
    reference[JOINT_NAMES[2]] = 0.21
    active, reason, error = desired_reference_gate(
        True, True, None, 0.01, 0.10, actual, reference, 0.20
    )
    assert not active
    assert reason == "reference_tracking_error_exceeded"
    assert error == 0.21

    reference[JOINT_NAMES[2]] = 0.19
    active, reason, error = desired_reference_gate(
        True, True, None, 0.01, 0.10, actual, reference, 0.20
    )
    assert active
    assert reason is None
    assert error == 0.19


class _FakeDynamics:
    def __init__(self):
        self.home_com = np.array([0.1, 0.0, 0.0])
        self.home_positions = {name: 0.0 for name in JOINT_NAMES}
        self.mass_property_calls = []
        self.state_calls = []

    def mass_properties(self, positions, payload):
        self.mass_property_calls.append((dict(positions), payload))
        return 1.3, np.array([0.3, 0.0, 0.0]), np.eye(3)

    def state(self, positions, velocities, accelerations, payload):
        self.state_calls.append(
            (
                dict(positions),
                dict(velocities),
                dict(accelerations),
                payload,
            )
        )
        return SimpleNamespace(
            reaction_force_body_n=np.array([1.0, 2.0, 3.0]),
            reaction_torque_body_nm=np.array([4.0, 5.0, 6.0]),
        )


def test_coupling_uses_actual_q_for_properties_and_reference_triple_for_dynamics():
    dynamics = _FakeDynamics()
    actual = {name: 0.01 for name in JOINT_NAMES}
    reference_q = {name: 0.02 for name in JOINT_NAMES}
    reference_qd = {name: 0.03 for name in JOINT_NAMES}
    reference_qdd = {name: 0.04 for name in JOINT_NAMES}
    result = evaluate_coupling_sources(
        dynamics,
        actual,
        reference_q,
        reference_qd,
        reference_qdd,
        True,
        None,
    )
    assert dynamics.mass_property_calls == [(actual, None)]
    assert dynamics.state_calls == [
        (reference_q, reference_qd, reference_qdd, None)
    ]
    assert np.allclose(result[3], [0.2, 0.0, 0.0])
    assert np.allclose(result[4], [1.0, 2.0, 3.0])
    assert np.allclose(result[5], [4.0, 5.0, 6.0])


def test_coupling_dynamic_failure_is_zero_without_measured_state_fallback():
    dynamics = _FakeDynamics()
    actual = {name: 0.01 for name in JOINT_NAMES}
    result = evaluate_coupling_sources(
        dynamics,
        actual,
        {name: 99.0 for name in JOINT_NAMES},
        {name: 99.0 for name in JOINT_NAMES},
        {name: 99.0 for name in JOINT_NAMES},
        False,
        None,
    )
    assert dynamics.mass_property_calls == [(actual, None)]
    assert dynamics.state_calls == []
    assert np.array_equal(result[4], np.zeros(3))
    assert np.array_equal(result[5], np.zeros(3))


def test_nonfinite_sample_does_not_poison_last_finite_joint_state():
    node = _bare_monitor()
    node.on_joint_state(
        _message([0.1] * len(JOINT_NAMES), [0.2] * len(JOINT_NAMES), 1.0)
    )
    accepted_positions = dict(node.positions)
    assert node.last_joint_sample_valid

    invalid_positions = [0.3] * len(JOINT_NAMES)
    invalid_positions[1] = float("nan")
    node.on_joint_state(
        _message(invalid_positions, [0.4] * len(JOINT_NAMES), 1.01)
    )

    assert not node.last_joint_sample_valid
    assert node.last_joint_invalid_reason == "non_finite_joint_position"
    assert node.positions == accepted_positions
    assert all(np.isfinite(value) for value in node.positions.values())


def test_motion_metrics_expose_velocity_and_filtered_acceleration_peaks():
    velocities = {
        name: value
        for name, value in zip(JOINT_NAMES, [0.1, -0.4, 0.2, 0.0, 0.3, -0.1])
    }
    accelerations = {
        name: value
        for name, value in zip(JOINT_NAMES, [0.2, 0.3, -1.25, 0.0, 0.1, 0.4])
    }
    assert joint_motion_metrics(velocities, accelerations) == {
        "maximum_joint_velocity_rad_s": 0.4,
        "filtered_joint_acceleration_peak_rad_s2": 1.25,
    }


def test_filter_recovers_from_nonfinite_previous_state():
    assert np.isfinite(
        filtered_joint_acceleration_step(float("nan"), 1.0, 0.01, 0.20, 4.0)
    )


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_stale_state_report_is_json_safe_and_contains_motion_gates():
    node = _bare_monitor()
    node.positions = {name: 0.0 for name in JOINT_NAMES}
    node.velocities = {name: 0.03 for name in JOINT_NAMES}
    node.accelerations = {name: 0.07 for name in JOINT_NAMES}
    node.last_joint_sample_valid = True
    node.last_joint_invalid_reason = None
    node.last_joint_receipt_monotonic_s = None
    node.state_publisher = _Publisher()
    node.last_log_time = 10.0

    with (
        patch.object(monitor_module.time, "monotonic", return_value=10.0),
        patch.object(
            monitor_module,
            "String",
            side_effect=lambda **values: SimpleNamespace(**values),
        ),
    ):
        node.on_timer()

    report = json.loads(node.state_publisher.messages[-1].data)
    assert report["estimator_valid"] is False
    assert report["source_fresh"] is False
    assert report["joint_state_valid"] is True
    assert report["invalid_reason"] == "joint_state_stale"
    assert report["source_age_s"] is None
    assert report["maximum_joint_velocity_rad_s"] == 0.03
    assert report["filtered_joint_acceleration_peak_rad_s2"] == 0.07


def test_fresh_nonfinite_sample_reports_invalid_without_reusing_compensation():
    node = _bare_monitor()
    node.positions = {name: 0.0 for name in JOINT_NAMES}
    node.velocities = {name: 0.0 for name in JOINT_NAMES}
    node.accelerations = {name: 0.0 for name in JOINT_NAMES}
    node.last_joint_sample_valid = False
    node.last_joint_invalid_reason = "non_finite_joint_velocity"
    node.last_joint_receipt_monotonic_s = 9.99
    node.state_publisher = _Publisher()
    node.last_log_time = 10.0

    with (
        patch.object(monitor_module.time, "monotonic", return_value=10.0),
        patch.object(
            monitor_module,
            "String",
            side_effect=lambda **values: SimpleNamespace(**values),
        ),
    ):
        node.on_timer()

    report = json.loads(node.state_publisher.messages[-1].data)
    assert report["source_fresh"] is True
    assert report["joint_state_valid"] is False
    assert report["estimator_valid"] is False
    assert report["invalid_reason"] == "non_finite_joint_velocity"


def test_nonfinite_cached_motion_state_is_published_as_json_null():
    node = _bare_monitor()
    node.positions = {name: 0.0 for name in JOINT_NAMES}
    node.velocities = {name: 0.0 for name in JOINT_NAMES}
    node.velocities[JOINT_NAMES[0]] = float("nan")
    node.accelerations = {name: 0.0 for name in JOINT_NAMES}
    node.last_joint_sample_valid = True
    node.last_joint_invalid_reason = None
    node.last_joint_receipt_monotonic_s = 9.99
    node.state_publisher = _Publisher()
    node.last_log_time = 10.0

    with (
        patch.object(monitor_module.time, "monotonic", return_value=10.0),
        patch.object(
            monitor_module,
            "String",
            side_effect=lambda **values: SimpleNamespace(**values),
        ),
    ):
        node.on_timer()

    report = json.loads(node.state_publisher.messages[-1].data)
    assert report["estimator_valid"] is False
    assert report["joint_state_valid"] is False
    assert report["invalid_reason"] == "non_finite_joint_motion_state"
    assert report["maximum_joint_velocity_rad_s"] is None
