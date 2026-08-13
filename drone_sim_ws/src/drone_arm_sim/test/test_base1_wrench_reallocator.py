import json
from pathlib import Path
import unittest

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.base1_wrench_reallocator import (
    allocate_total_wrench,
    commands_to_config_thrust_n,
    compensation_wrench_frd,
    config_order_to_motor_order,
    config_thrust_n_to_commands,
    flight_state_allows_compensation,
    motor_order_to_config_order,
    position_feedback_force_frd,
    quaternion_xyzw_to_rotation_body_to_world,
    relative_gravity_wrench,
    slew_vector,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = json.loads(
    (PACKAGE / "config/my_drone_v3_cad_debug_4kg.json").read_text(encoding="utf-8")
)


class Base1WrenchReallocatorTest(unittest.TestCase):
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
        matrix = allocation_matrix(CONFIG, position_key="wrench_position_m")
        expected = matrix @ commands_to_config_thrust_n(CONFIG, commands)
        np.testing.assert_allclose(result["realized_wrench_frd"], expected, atol=1e-9)
        np.testing.assert_allclose(result["commands_motor_order"], commands, atol=1e-9)

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
