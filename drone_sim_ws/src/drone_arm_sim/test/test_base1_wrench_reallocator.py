import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix
import drone_arm_sim.base1_wrench_reallocator as reallocator_module
from drone_arm_sim.base1_wrench_reallocator import (
    Base1WrenchReallocator,
    WrenchEffortBaseline,
    adaptive_application_gate,
    adaptive_quasi_static_gate,
    allocate_feasible_compensation,
    allocate_total_wrench,
    clear_revoked_direct_xy_force,
    classify_coupling_state_sample,
    commands_to_config_thrust_n,
    compensation_wrench_frd,
    config_order_to_motor_order,
    config_thrust_n_to_commands,
    direct_xy_force_command_is_fresh,
    direct_xy_protocol_phase,
    flight_state_allows_compensation,
    motor_order_to_config_order,
    position_feedback_force_frd,
    quaternion_xyzw_to_rotation_body_to_world,
    relative_gravity_wrench,
    slew_vector,
    unconstrained_allocation_delta_map,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = json.loads(
    (PACKAGE / "config/my_drone_v3_cad_debug_4kg.json").read_text(encoding="utf-8")
)


class Base1WrenchReallocatorTest(unittest.TestCase):
    def test_single_joint_state_stale_frame_holds_last_valid_inside_source_lease(self):
        state = {
            "estimator_valid": False,
            "source_fresh": False,
            "joint_state_valid": False,
            "invalid_reason": "joint_state_stale",
        }
        status, reason, velocity, acceleration = classify_coupling_state_sample(
            state,
            now_s=10.11,
            last_valid_stamp_s=10.0,
            hold_timeout_s=0.5,
        )
        self.assertEqual(status, "held_last_valid")
        self.assertEqual(reason, "joint_state_stale")
        self.assertIsNone(velocity)
        self.assertIsNone(acceleration)

    def test_persistent_joint_state_stale_expires_at_existing_source_lease(self):
        state = {
            "estimator_valid": False,
            "source_fresh": False,
            "joint_state_valid": False,
            "invalid_reason": "joint_state_stale",
        }
        status, reason, _, _ = classify_coupling_state_sample(
            state,
            now_s=10.500001,
            last_valid_stamp_s=10.0,
            hold_timeout_s=0.5,
        )
        self.assertEqual(status, "rejected")
        self.assertEqual(reason, "joint_state_stale")

    def test_corrupt_coupling_state_never_uses_hold_last_valid(self):
        state = {
            "estimator_valid": False,
            "source_fresh": True,
            "joint_state_valid": False,
            "invalid_reason": "non_finite_joint_motion_state",
        }
        status, reason, _, _ = classify_coupling_state_sample(
            state,
            now_s=10.01,
            last_valid_stamp_s=10.0,
            hold_timeout_s=0.5,
        )
        self.assertEqual(status, "rejected")
        self.assertEqual(reason, "non_finite_joint_motion_state")

    def test_valid_coupling_state_requires_finite_motion_metrics(self):
        state = {
            "estimator_valid": True,
            "source_fresh": True,
            "joint_state_valid": True,
            "maximum_joint_velocity_rad_s": float("nan"),
            "filtered_joint_acceleration_peak_rad_s2": 0.1,
        }
        status, reason, _, _ = classify_coupling_state_sample(
            state,
            now_s=10.01,
            last_valid_stamp_s=10.0,
            hold_timeout_s=0.5,
        )
        self.assertEqual(status, "rejected")
        self.assertEqual(reason, "non_finite_joint_motion_metrics")

    def test_compact_state_exposes_direct_xy_ownership_health_contract(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = object.__new__(Base1WrenchReallocator)
        node.last_diagnostic_s = 0.0
        node.diagnostic_period_s = 0.0
        node.position_feedback_enabled = True
        node.position_target_world_enu = np.zeros(3)
        node.direct_xy_force_enabled = True
        node.direct_xy_force_epoch = 4
        node.direct_xy_minimum_enable_epoch = 4
        node.direct_xy_force_stamp_s = 1.0
        node.direct_xy_force_timeout_s = 0.2
        node.last_allocation_limited = False
        node.source_timeout_s = 0.5
        node.valid_state_stamp_s = 1.0
        node.coupling_state_receive_stamp_s = 1.0
        node.coupling_state_status = "held_last_valid"
        node.coupling_state_invalid_reason = "joint_state_stale"
        node.reaction_force_gain = 0.0
        node.reaction_torque_gain = 0.0
        node.gravity_torque_gain = 1.0
        node.gravity_stamp_s = 1.0
        node.reaction_stamp_s = None
        node.diagnostic_publisher = Recorder()
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.String", SimpleNamespace
        ):
            node._publish_diagnostic_state(
                1.0,
                event="allocated",
                source_fresh=True,
                flight_allowed=True,
                headroom_ok=True,
                motion_active=True,
                input_commands=np.full(8, 0.5),
                output_commands=np.full(8, 0.5),
                requested_wrench_frd=np.zeros(6),
                delivered_wrench_frd=np.zeros(6),
                feasibility_scale=1.0,
                residual_norm=0.0,
                saturated=0,
                truth_fresh=True,
                position_feedback_active=True,
                position_feedback_prepared=True,
            )
        report = json.loads(node.diagnostic_publisher.messages[-1].data)
        self.assertTrue(report["position_feedback_enabled"])
        self.assertTrue(report["position_feedback_ready"])
        self.assertTrue(report["position_feedback_active"])
        self.assertTrue(report["position_feedback_prepared"])
        self.assertEqual(report["direct_xy_protocol_phase"], "active")
        self.assertTrue(report["direct_xy_force_command_fresh"])
        self.assertEqual(report["direct_xy_force_epoch"], 4)
        self.assertTrue(report["position_target_latched"])
        self.assertTrue(report["truth_fresh"])
        self.assertFalse(report["allocation_limited"])
        self.assertEqual(report["coupling_state_status"], "held_last_valid")
        self.assertEqual(report["coupling_state_invalid_reason"], "joint_state_stale")
        self.assertTrue(report["coupling_state_fresh"])
        self.assertTrue(report["gravity_wrench_fresh"])
        self.assertEqual(report["source_invalid_reasons"], [])

    def test_direct_xy_protocol_transition_bypasses_diagnostic_period(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = object.__new__(Base1WrenchReallocator)
        node.last_diagnostic_s = 1.0
        node.diagnostic_period_s = 1.0
        node.last_diagnostic_protocol_signature = (
            "allocated", True, True, False, False, 2, True
        )
        node.position_feedback_enabled = True
        node.position_target_world_enu = np.zeros(3)
        node.direct_xy_force_enabled = True
        node.direct_xy_force_epoch = 2
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = 1.0
        node.direct_xy_force_timeout_s = 0.2
        node.last_allocation_limited = False
        node.diagnostic_publisher = Recorder()
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.String", SimpleNamespace
        ):
            node._publish_diagnostic_state(
                1.01,
                event="allocated",
                source_fresh=True,
                flight_allowed=True,
                headroom_ok=True,
                motion_active=True,
                input_commands=np.full(8, 0.5),
                output_commands=np.full(8, 0.5),
                requested_wrench_frd=np.zeros(6),
                delivered_wrench_frd=np.zeros(6),
                feasibility_scale=1.0,
                residual_norm=0.0,
                saturated=0,
                truth_fresh=True,
                position_feedback_active=True,
                position_feedback_prepared=True,
            )
        self.assertEqual(len(node.diagnostic_publisher.messages), 1)
        report = json.loads(node.diagnostic_publisher.messages[0].data)
        self.assertEqual(report["direct_xy_protocol_phase"], "active")

    def test_fixed_rate_refresh_is_the_single_periodic_producer(self):
        node = object.__new__(Base1WrenchReallocator)
        node.enabled = True
        node.latest_command_message = object()
        node.diagnostic_period_s = 0.01
        node.last_command_s = 10.0
        calls = []
        node._process_command = lambda message, *, now_s: calls.append(
            (message, now_s)
        )

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.011,
        ):
            node._on_command_refresh_timer()
        self.assertEqual(calls, [(node.latest_command_message, 10.011)])

    def test_fixed_rate_refresh_cannot_emit_without_a_valid_cached_level(self):
        node = object.__new__(Base1WrenchReallocator)
        node.enabled = True
        node.latest_command_message = None
        node.diagnostic_period_s = 0.01
        node.last_command_s = None
        node._process_command = unittest.mock.Mock()
        node._on_command_refresh_timer()
        node._process_command.assert_not_called()

    def test_enabled_input_callback_only_replaces_latest_valid_level(self):
        node = object.__new__(Base1WrenchReallocator)
        node.enabled = True
        node.velocity_scale = 1000.0
        node.latest_command_message = None
        node.publisher = unittest.mock.Mock()
        node._process_command = unittest.mock.Mock()
        message = SimpleNamespace(
            normalized=[0.25] * 8,
            velocity=[],
        )
        node.on_command(message)
        self.assertIsNot(node.latest_command_message, message)
        self.assertEqual(node.latest_command_message.normalized, [0.25] * 8)
        node._process_command.assert_not_called()
        node.publisher.publish.assert_not_called()

    def test_direct_xy_protocol_is_prepare_then_independent_enable(self):
        node = object.__new__(Base1WrenchReallocator)
        node.truth_position_world_enu = np.asarray([1.0, 2.0, 3.0])
        node.arm_motion_active = False
        node.arm_motion_stamp_s = None
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = None
        node.direct_xy_force_enabled = False
        node.direct_xy_force_epoch = -1
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.0,
        ):
            node.on_arm_motion(SimpleNamespace(data=True))
        np.testing.assert_array_equal(
            node.position_target_world_enu, node.truth_position_world_enu
        )
        self.assertFalse(node.direct_xy_force_enabled)

        command = SimpleNamespace(data=json.dumps({
            "schema": "my_drone.arm-direct-xy-force-command.v1",
            "ownership_epoch": 3,
            "enabled": True,
        }))
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.01,
        ):
            node.on_direct_xy_force_command(command)
        self.assertTrue(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_epoch, 3)

        disable = SimpleNamespace(data=json.dumps({
            "schema": "my_drone.arm-direct-xy-force-command.v1",
            "ownership_epoch": 3,
            "enabled": False,
        }))
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.02,
        ):
            node.on_direct_xy_force_command(disable)
        self.assertFalse(node.direct_xy_force_enabled)
        self.assertTrue(node.direct_xy_force_clear_pending)

    def test_new_motion_rejects_delayed_grant_from_previous_epoch(self):
        node = object.__new__(Base1WrenchReallocator)
        node.truth_position_world_enu = np.zeros(3)
        node.arm_motion_active = False
        node.arm_motion_stamp_s = None
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = None
        node.direct_xy_force_enabled = False
        node.direct_xy_force_epoch = 7
        node.direct_xy_minimum_enable_epoch = 7
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=20.0,
        ):
            node.on_arm_motion(SimpleNamespace(data=True))
        self.assertEqual(node.direct_xy_minimum_enable_epoch, 8)

        delayed = SimpleNamespace(data=json.dumps({
            "schema": "my_drone.arm-direct-xy-force-command.v1",
            "ownership_epoch": 7,
            "enabled": True,
        }))
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=20.01,
        ):
            node.on_direct_xy_force_command(delayed)
        self.assertFalse(node.direct_xy_force_enabled)

        current = SimpleNamespace(data=json.dumps({
            "schema": "my_drone.arm-direct-xy-force-command.v1",
            "ownership_epoch": 8,
            "enabled": True,
        }))
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=20.02,
        ):
            node.on_direct_xy_force_command(current)
        self.assertTrue(node.direct_xy_force_enabled)

    def test_force_lease_generation_rejects_stale_true_after_newer_false(self):
        node = object.__new__(Base1WrenchReallocator)
        node.arm_motion_active = True
        node.arm_motion_stamp_s = 10.0
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = np.zeros(3)
        node.direct_xy_force_enabled = False
        node.direct_xy_force_epoch = -1
        node.direct_xy_force_lease_generation = -1
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False

        def command(enabled, generation):
            return SimpleNamespace(data=json.dumps({
                "schema": "my_drone.arm-direct-xy-force-command.v1",
                "ownership_epoch": 3,
                "enabled": enabled,
                "lease_generation": generation,
            }))

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.01,
        ):
            node.on_direct_xy_force_command(command(True, 5))
        self.assertTrue(node.direct_xy_force_enabled)

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.02,
        ):
            node.on_direct_xy_force_command(command(False, 6))
        self.assertFalse(node.direct_xy_force_enabled)

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=10.03,
        ):
            node.on_direct_xy_force_command(command(True, 5))
        self.assertFalse(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_lease_generation, 6)

    def test_guardian_session_requires_disable_handshake_and_is_echoed(self):
        node = object.__new__(Base1WrenchReallocator)
        node.direct_xy_force_enabled = False
        node.direct_xy_force_session_id = ""
        node.direct_xy_force_epoch = -1
        node.direct_xy_force_lease_generation = -1
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False
        node.arm_motion_active = True
        node.arm_motion_stamp_s = 1.0
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = np.zeros(3)

        def command(enabled, generation):
            return SimpleNamespace(
                data=json.dumps(
                    {
                        "schema": "my_drone.arm-direct-xy-force-command.v1",
                        "controller_session_id": "boot-a",
                        "ownership_epoch": 1,
                        "enabled": enabled,
                        "lease_generation": generation,
                    }
                )
            )

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=1.1,
        ):
            node.on_direct_xy_force_command(command(True, 1))
        self.assertFalse(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_session_id, "")

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=1.1,
        ):
            node.on_direct_xy_force_command(command(False, 1))
        self.assertEqual(node.direct_xy_force_session_id, "boot-a")
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=1.2,
        ):
            node.on_direct_xy_force_command(command(True, 2))
        self.assertTrue(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_lease_generation, 2)

    def test_retired_guardian_session_cannot_be_revived_by_delayed_messages(self):
        node = object.__new__(Base1WrenchReallocator)
        node.direct_xy_force_enabled = False
        node.direct_xy_force_session_id = "boot-a"
        node.direct_xy_retired_sessions = set()
        node.direct_xy_force_epoch = 2
        node.direct_xy_force_lease_generation = 7
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False
        node.arm_motion_active = True
        node.arm_motion_stamp_s = 1.0
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = np.zeros(3)

        def command(session, enabled, generation, epoch=3):
            return SimpleNamespace(
                data=json.dumps(
                    {
                        "schema": "my_drone.arm-direct-xy-force-command.v1",
                        "controller_session_id": session,
                        "ownership_epoch": epoch,
                        "enabled": enabled,
                        "lease_generation": generation,
                    }
                )
            )

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=1.1,
        ):
            node.on_direct_xy_force_command(command("boot-b", False, 1))
            node.on_direct_xy_force_command(command("boot-b", True, 2))
        self.assertTrue(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_session_id, "boot-b")

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.time.monotonic",
            return_value=1.2,
        ):
            node.on_direct_xy_force_command(command("boot-a", False, 8))
            node.on_direct_xy_force_command(command("boot-a", True, 9))
        self.assertTrue(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_session_id, "boot-b")
        self.assertEqual(node.direct_xy_force_lease_generation, 2)

    def test_guardian_force_command_rejects_non_boolean_enabled(self):
        node = object.__new__(Base1WrenchReallocator)
        node.direct_xy_force_enabled = False
        node.direct_xy_force_session_id = "boot-a"
        node.direct_xy_retired_sessions = set()
        node.direct_xy_force_epoch = 0
        node.direct_xy_force_lease_generation = 0
        node.direct_xy_minimum_enable_epoch = 0
        node.direct_xy_force_stamp_s = None
        node.direct_xy_force_clear_pending = False
        node.position_feedback_was_active = False
        node.arm_motion_active = True
        node.arm_motion_stamp_s = 1.0
        node.arm_motion_timeout_s = 1.0
        node.position_target_world_enu = np.zeros(3)
        message = SimpleNamespace(
            data=json.dumps(
                {
                    "schema": "my_drone.arm-direct-xy-force-command.v1",
                    "controller_session_id": "boot-a",
                    "ownership_epoch": 1,
                    "enabled": "false",
                    "lease_generation": 1,
                }
            )
        )
        node.on_direct_xy_force_command(message)
        self.assertFalse(node.direct_xy_force_enabled)
        self.assertEqual(node.direct_xy_force_lease_generation, 0)

    def test_direct_xy_force_grant_freshness_and_phase_fail_closed(self):
        self.assertTrue(direct_xy_force_command_is_fresh(True, 4.9, 5.0, 0.2))
        self.assertFalse(direct_xy_force_command_is_fresh(False, 4.9, 5.0, 0.2))
        self.assertFalse(direct_xy_force_command_is_fresh(True, 4.0, 5.0, 0.2))
        self.assertFalse(direct_xy_force_command_is_fresh(True, None, 5.0, 0.2))
        self.assertEqual(
            direct_xy_protocol_phase(
                position_feedback_enabled=True,
                motion_active=True,
                target_latched=True,
                prepared=True,
                force_active=False,
            ),
            "prepared",
        )
        self.assertEqual(
            direct_xy_protocol_phase(
                position_feedback_enabled=True,
                motion_active=True,
                target_latched=True,
                prepared=True,
                force_active=True,
            ),
            "active",
        )
        self.assertEqual(
            direct_xy_protocol_phase(
                position_feedback_enabled=False,
                motion_active=True,
                target_latched=True,
                prepared=True,
                force_active=True,
            ),
            "disabled",
        )

    def test_direct_xy_disable_clears_xy_without_erasing_other_compensation(self):
        current = np.asarray([0.18, -0.12, 0.04, 0.01, -0.02, 0.03])
        non_direct_target = np.asarray([0.02, 0.01, -0.05, 0.04, 0.05, -0.06])
        cleared = clear_revoked_direct_xy_force(
            current, non_direct_target, revoke=True
        )
        np.testing.assert_array_equal(cleared[:2], non_direct_target[:2])
        np.testing.assert_array_equal(cleared[2:], current[2:])
        np.testing.assert_array_equal(
            clear_revoked_direct_xy_force(current, non_direct_target, revoke=False),
            current,
        )

    def test_adaptive_path_does_not_reintroduce_pose_pd_controller(self):
        source = (PACKAGE / "drone_arm_sim/base1_wrench_reallocator.py").read_text(
            encoding="utf-8"
        )
        adaptive_source = (PACKAGE / "drone_arm_sim/slow_adaptive_wrench.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("restoring_wrench_frd", source + adaptive_source)
        self.assertNotIn("adaptive-position-gain", source)
        self.assertIn("raw_wrench_frd - adaptive_baseline_frd", source)

    def test_wrench_effort_baseline_requires_one_continuous_stable_window(self):
        baseline = WrenchEffortBaseline(hold_s=0.25)
        wrench = np.asarray([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        self.assertIsNone(baseline.step(wrench, 0.1, eligible=True))
        self.assertIsNone(baseline.step(wrench, 0.1, eligible=False))
        self.assertEqual(baseline.duration_s, 0.0)
        self.assertIsNone(baseline.step(wrench, 0.1, eligible=True))
        self.assertIsNone(baseline.step(wrench, 0.1, eligible=True))
        value = baseline.step(wrench, 0.1, eligible=True)
        np.testing.assert_allclose(value, wrench, atol=1.0e-12)
        np.testing.assert_allclose(
            baseline.step(2.0 * wrench, 0.1, eligible=False), wrench, atol=1.0e-12
        )

    def test_wrench_effort_baseline_invalid_dt_fails_closed_without_nan_state(self):
        baseline = WrenchEffortBaseline(hold_s=0.25)
        self.assertIsNone(baseline.step(np.ones(6), float("nan"), eligible=True))
        self.assertEqual(baseline.duration_s, 0.0)
        np.testing.assert_array_equal(baseline.integral, np.zeros(6))

    def test_adaptive_quasi_static_gate_is_vector_norm_bounded_and_fail_closed(self):
        keyword = dict(
            joint_velocity_limit_rad_s=0.005,
            joint_acceleration_limit_rad_s2=0.02,
            body_speed_limit_m_s=0.03,
            angular_rate_limit_rad_s=0.01,
        )
        self.assertTrue(
            adaptive_quasi_static_gate(
                0.004,
                0.01,
                np.asarray([0.01, 0.01, 0.0]),
                np.asarray([0.001, 0.002, 0.0]),
                **keyword,
            )
        )
        self.assertFalse(
            adaptive_quasi_static_gate(
                0.004,
                0.01,
                np.asarray([0.025, 0.025, 0.0]),
                np.zeros(3),
                **keyword,
            )
        )
        self.assertFalse(
            adaptive_quasi_static_gate(
                None, 0.01, np.zeros(3), np.zeros(3), **keyword
            )
        )

    def test_retained_adaptive_state_is_applied_only_behind_all_safety_gates(self):
        self.assertTrue(adaptive_application_gate(True, True, True, True, True))
        for index in range(5):
            gates = [True] * 5
            gates[index] = False
            self.assertFalse(adaptive_application_gate(*gates))

    def test_motor_order_round_trip_preserves_px4_numbering(self):
        values = np.arange(1.0, 9.0)
        in_config_order = motor_order_to_config_order(CONFIG, values)
        np.testing.assert_array_equal(
            config_order_to_motor_order(CONFIG, in_config_order), values
        )

    def test_command_thrust_round_trip(self):
        commands = np.linspace(0.15, 0.85, 8)
        thrust = commands_to_config_thrust_n(CONFIG, commands)
        np.testing.assert_allclose(
            config_thrust_n_to_commands(CONFIG, thrust), commands, atol=1.0e-12
        )

    def test_zero_compensation_reconstructs_base_wrench(self):
        commands = np.asarray([0.36, 0.39, 0.41, 0.38, 0.40, 0.37, 0.42, 0.35])
        result = allocate_total_wrench(
            CONFIG, commands, np.zeros(6), maximum_motor_delta_n=0.5
        )
        matrix = allocation_matrix(CONFIG, position_key="position_m")
        expected = matrix @ commands_to_config_thrust_n(CONFIG, commands)
        np.testing.assert_allclose(result["realized_wrench_frd"], expected, atol=1e-9)
        np.testing.assert_allclose(result["commands_motor_order"], commands, atol=1e-9)

    def test_compensation_wrench_is_about_vehicle_com(self):
        commands = np.full(8, 0.42)
        compensation = np.asarray([0.05, -0.04, 0.0, 0.0, 0.0, 0.0])
        result = allocate_total_wrench(
            CONFIG, commands, compensation, maximum_motor_delta_n=0.5
        )
        com_matrix = allocation_matrix(CONFIG, position_key="position_m")
        origin_matrix = allocation_matrix(CONFIG, position_key="wrench_position_m")
        base_thrust = commands_to_config_thrust_n(CONFIG, commands)
        delta_thrust = result["thrust_config_order_n"] - base_thrust
        delta_com = com_matrix @ delta_thrust
        delta_origin = origin_matrix @ delta_thrust
        first_rotor = CONFIG["rotors"][0]
        center = (
            np.asarray(first_rotor["wrench_position_m"])
            - np.asarray(first_rotor["position_m"])
        )
        # Transform the physically published origin wrench back to the CoM.
        origin_as_com = delta_origin.copy()
        origin_as_com[3:] -= np.cross(center, delta_origin[:3])
        np.testing.assert_allclose(delta_com, compensation, atol=1e-6)
        np.testing.assert_allclose(origin_as_com, compensation, atol=1e-6)

    def test_total_wrench_is_single_bounded_allocation(self):
        commands = np.full(8, 0.42)
        compensation = np.asarray([0.05, -0.04, 0.02, 0.01, -0.015, 0.008])
        result = allocate_total_wrench(
            CONFIG, commands, compensation, maximum_motor_delta_n=0.5
        )
        self.assertTrue(result["success"])
        np.testing.assert_allclose(
            result["desired_wrench_frd"] - result["base_wrench_frd"],
            compensation,
            atol=1e-12,
        )
        self.assertLess(result["residual_norm"], 1.0e-6)
        self.assertLessEqual(
            float(
                np.max(
                    np.abs(
                        result["thrust_config_order_n"]
                        - result["base_thrust_config_order_n"]
                    )
                )
            ),
            0.5 + 1e-9,
        )

    def test_fast_interior_allocation_avoids_bounded_solver_and_is_exact(self):
        commands = np.full(8, 0.42)
        compensation = np.asarray([0.02, -0.015, 0.01, 0.004, -0.006, 0.003])
        delta_map = unconstrained_allocation_delta_map(CONFIG)
        matrix = allocation_matrix(CONFIG, position_key="position_m")
        np.testing.assert_allclose(matrix @ delta_map, np.eye(6), atol=1.0e-10)

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.lsq_linear",
            side_effect=AssertionError("interior fast path called bounded solver"),
        ) as bounded_solver:
            result = allocate_total_wrench(
                CONFIG,
                commands,
                compensation,
                maximum_motor_delta_n=0.5,
                allocation_delta_map=delta_map,
            )

        bounded_solver.assert_not_called()
        base_thrust = commands_to_config_thrust_n(CONFIG, commands)
        expected_thrust = base_thrust + delta_map @ compensation
        np.testing.assert_allclose(
            result["thrust_config_order_n"], expected_thrust, atol=1.0e-12
        )
        np.testing.assert_allclose(
            result["commands_motor_order"],
            config_thrust_n_to_commands(CONFIG, expected_thrust),
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            result["realized_wrench_frd"] - result["base_wrench_frd"],
            compensation,
            atol=1.0e-10,
        )
        np.testing.assert_allclose(result["residual_frd"], np.zeros(6), atol=1.0e-10)
        self.assertLess(result["residual_norm"], 1.0e-10)
        self.assertTrue(result["success"])

    def test_fast_allocation_falls_back_to_bounded_solver_outside_bounds(self):
        commands = np.full(8, 0.48)
        compensation = np.asarray([0.0, 0.0, 0.0, 0.0, 2.5, 0.0])
        delta_limit = 0.20
        delta_map = unconstrained_allocation_delta_map(CONFIG)
        base_thrust = commands_to_config_thrust_n(CONFIG, commands)
        maximum = float(CONFIG["maximum_thrust_n"])
        lower = np.maximum(0.0, base_thrust - delta_limit)
        upper = np.minimum(maximum, base_thrust + delta_limit)
        unconstrained = base_thrust + delta_map @ compensation
        self.assertTrue(
            np.any(unconstrained < lower) or np.any(unconstrained > upper),
            "test target must leave the fast-path interior",
        )

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.lsq_linear",
            wraps=reallocator_module.lsq_linear,
        ) as bounded_solver:
            result = allocate_total_wrench(
                CONFIG,
                commands,
                compensation,
                maximum_motor_delta_n=delta_limit,
                allocation_delta_map=delta_map,
            )

        bounded_solver.assert_called_once()
        thrust = result["thrust_config_order_n"]
        self.assertTrue(np.all(thrust >= lower - 1.0e-12))
        self.assertTrue(np.all(thrust <= upper + 1.0e-12))
        matrix = allocation_matrix(CONFIG, position_key="position_m")
        np.testing.assert_allclose(
            result["realized_wrench_frd"], matrix @ thrust, atol=1.0e-12
        )
        np.testing.assert_allclose(
            result["residual_frd"],
            result["realized_wrench_frd"] - result["desired_wrench_frd"],
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            result["residual_norm"],
            float(np.linalg.norm(result["residual_frd"])),
            places=12,
        )
        self.assertTrue(result["success"])

    def test_fast_allocation_preserves_nondefault_regularization_semantics(self):
        commands = np.full(8, 0.42)
        compensation = np.asarray([0.02, -0.015, 0.01, 0.004, -0.006, 0.003])
        delta_map = unconstrained_allocation_delta_map(CONFIG)

        with patch(
            "drone_arm_sim.base1_wrench_reallocator.lsq_linear",
            wraps=reallocator_module.lsq_linear,
        ) as bounded_solver:
            result = allocate_total_wrench(
                CONFIG,
                commands,
                compensation,
                maximum_motor_delta_n=0.5,
                regularization=1.0e-3,
                allocation_delta_map=delta_map,
            )

        bounded_solver.assert_called_once()
        self.assertTrue(result["success"])

    def test_motor_callback_contains_no_synchronous_legacy_state_log(self):
        source = (PACKAGE / "drone_arm_sim/base1_wrench_reallocator.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("BASE1_COMPENSATION_STATE", source)
        self.assertIn("ReliabilityPolicy.BEST_EFFORT", source)
        self.assertIn("HistoryPolicy.KEEP_LAST", source)
        self.assertIn("depth=1", source)

    def test_direct_xy_force_reader_uses_reliable_latest_qos(self):
        source = (PACKAGE / "drone_arm_sim/base1_wrench_reallocator.py").read_text(
            encoding="utf-8"
        )
        profile_start = source.index("direct_xy_safety_qos = QoSProfile(")
        subscription_end = source.index("px4_qos = QoSProfile(", profile_start)
        direct_xy_section = source[profile_start:subscription_end]

        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", direct_xy_section)
        self.assertIn("history=HistoryPolicy.KEEP_LAST", direct_xy_section)
        self.assertIn("depth=1", direct_xy_section)
        self.assertIn('"/my_drone/arm_direct_xy_force_command"', direct_xy_section)
        self.assertIn("direct_xy_safety_qos", direct_xy_section)

    def test_fast_allocation_rejects_wrong_matrix_shapes(self):
        with patch(
            "drone_arm_sim.base1_wrench_reallocator.allocation_matrix",
            return_value=np.zeros((6, 7)),
        ):
            with self.assertRaisesRegex(
                ValueError, "allocation matrix must be finite and 6x8"
            ):
                unconstrained_allocation_delta_map(CONFIG)

        with self.assertRaisesRegex(
            ValueError, "allocation_delta_map must be finite and 8x6"
        ):
            allocate_total_wrench(
                CONFIG,
                np.full(8, 0.42),
                np.zeros(6),
                maximum_motor_delta_n=0.5,
                allocation_delta_map=np.zeros((8, 5)),
            )

    def test_infeasible_compensation_is_scaled_not_cleared(self):
        commands = np.full(8, 0.48)
        requested = np.asarray([0.0, 0.0, 0.0, 0.0, 2.5, 0.0])
        result = allocate_feasible_compensation(
            CONFIG,
            commands,
            requested,
            maximum_motor_delta_n=0.70,
            maximum_residual_norm=0.02,
        )
        self.assertTrue(result["limited"])
        self.assertGreater(result["feasibility_scale"], 0.0)
        self.assertLess(result["feasibility_scale"], 1.0)
        delivered = result["delivered_compensation_wrench_frd"]
        self.assertGreater(float(np.linalg.norm(delivered)), 0.1)
        np.testing.assert_allclose(
            delivered,
            result["feasibility_scale"] * requested,
            atol=1.0e-12,
        )
        self.assertTrue(result["allocation"]["success"])
        self.assertLessEqual(result["allocation"]["residual_norm"], 0.02)

    def test_feasible_compensation_is_not_scaled(self):
        commands = np.full(8, 0.48)
        requested = np.asarray([0.01, -0.02, 0.01, 0.01, 0.10, -0.01])
        result = allocate_feasible_compensation(
            CONFIG,
            commands,
            requested,
            maximum_motor_delta_n=1.60,
            maximum_residual_norm=0.02,
        )
        self.assertFalse(result["limited"])
        self.assertEqual(result["feasibility_scale"], 1.0)
        np.testing.assert_array_equal(
            result["delivered_compensation_wrench_frd"], requested
        )

    def test_disturbance_sign_and_flu_to_frd_conversion(self):
        reaction = np.asarray([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        gravity = np.asarray([0.0, 0.0, 0.0, 0.4, 0.5, 0.6])
        result = compensation_wrench_frd(
            reaction,
            gravity,
            reaction_force_gain=0.1,
            reaction_torque_gain=0.1,
            gravity_torque_gain=0.05,
            force_limit_n=10.0,
            reaction_torque_limit_nm=10.0,
            gravity_torque_limit_nm=10.0,
        )
        np.testing.assert_allclose(
            result, [-0.1, 0.2, 0.3, -0.03, 0.045, 0.06], atol=1e-12
        )

    def test_gravity_compensation_is_incremental_from_takeoff_trim(self):
        reference = np.asarray([0.0, 0.0, 0.0, 0.04, -0.01, 0.0])
        current = np.asarray([0.0, 0.0, 0.0, 0.16, 0.02, -0.01])
        np.testing.assert_array_equal(
            relative_gravity_wrench(current, reference),
            np.asarray([0.0, 0.0, 0.0, 0.12, 0.03, -0.01]),
        )
        np.testing.assert_array_equal(
            relative_gravity_wrench(reference, reference), np.zeros(6)
        )
        np.testing.assert_array_equal(
            relative_gravity_wrench(current, None), np.zeros(6)
        )

    def test_stale_target_slews_to_zero_without_freezing(self):
        current = np.asarray([0.2, -0.1, 0.05, 0.02, -0.01, 0.03])
        first = slew_vector(current, np.zeros(6), 0.1, 0.5, 0.1)
        second = slew_vector(first, np.zeros(6), 1.0, 0.5, 0.1)
        self.assertLess(np.linalg.norm(first), np.linalg.norm(current))
        np.testing.assert_array_equal(second, np.zeros(6))

    def test_flight_state_gate_fails_closed(self):
        self.assertTrue(flight_state_allows_compensation(True, True, 0.1, 0.5))
        self.assertFalse(flight_state_allows_compensation(False, True, 0.1, 0.5))
        self.assertFalse(flight_state_allows_compensation(True, False, 0.1, 0.5))
        self.assertFalse(flight_state_allows_compensation(True, True, 0.6, 0.5))
        self.assertFalse(
            flight_state_allows_compensation(True, True, float("inf"), 0.5)
        )

    def test_position_feedback_force_is_bounded_and_converted_to_frd(self):
        force = position_feedback_force_frd(
            np.asarray([0.0, 0.0, 1.0]),
            np.asarray([0.10, -0.10, 0.90]),
            np.asarray([0.02, -0.01, 0.03]),
            np.eye(3),
            position_gain_n_m=4.0,
            velocity_gain_n_s_m=2.0,
            horizontal_limit_n=0.20,
            vertical_limit_n=0.15,
        )
        # Raw ENU force is [-.44, .42, .34], then horizontal/vertical limits
        # apply before FLU [x,y,z] -> FRD [x,-y,-z].
        self.assertAlmostEqual(float(np.linalg.norm(force[:2])), 0.20, places=12)
        self.assertLess(force[0], 0.0)
        self.assertLess(force[1], 0.0)
        self.assertAlmostEqual(force[2], -0.15, places=12)

    def test_position_feedback_rotates_world_force_into_body(self):
        # +90 degree yaw maps body +X to world +Y.
        q = np.asarray([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
        rotation = quaternion_xyzw_to_rotation_body_to_world(q)
        np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)
        force = position_feedback_force_frd(
            np.asarray([0.0, 1.0, 0.0]),
            np.zeros(3),
            np.zeros(3),
            rotation,
            position_gain_n_m=1.0,
            velocity_gain_n_s_m=0.0,
            horizontal_limit_n=2.0,
            vertical_limit_n=1.0,
        )
        np.testing.assert_allclose(force, [1.0, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
