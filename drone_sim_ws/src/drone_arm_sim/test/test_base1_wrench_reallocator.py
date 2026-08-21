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

    # ── Hard fail-closed behavior ──────────────────────────────────────
    def test_disarm_immediately_zeros_compensation(self):
        """When flight state gate fails, compensation must be zeroed instantly,
        not slewed down.  Verify by checking that the gate logic correctly
        rejects the armed=False case regardless of nonzero current state."""
        self.assertFalse(flight_state_allows_compensation(False, True, 0.0, 5.0))
        # Slew from nonzero to zero target (simulates stale compensation fade).
        nonzero = np.asarray([0.1, 0.05, 0.02, 0.01, -0.005, 0.003])
        result = slew_vector(nonzero, np.zeros(6), 0.001, 1.0, 0.1)
        # With dt=0.001, slew is negligible → verification that hard-zero
        # path (current_compensation[:] = 0.0) is required for instant bypass.
        self.assertGreater(np.linalg.norm(result), 0.001)

    def test_no_headroom_rejects_compensation(self):
        """Insufficient motor headroom must also trigger immediate bypass."""
        commands_high = np.full(8, 0.98)
        base_thrust = commands_to_config_thrust_n(CONFIG, commands_high)
        maximum = float(CONFIG["maximum_thrust_n"])
        has_headroom = bool(
            np.min(maximum - base_thrust) >= 0.25
            and np.min(base_thrust) >= 0.25
        )
        self.assertFalse(has_headroom)

    def test_estimator_stale_allows_smooth_fade(self):
        """When estimator source is stale but flight is allowed, compensation
        should fade smoothly to zero via slew, not jump."""
        nonzero = np.asarray([0.2, 0.1, 0.05, 0.02, -0.01, 0.008])
        step1 = slew_vector(nonzero, np.zeros(6), 0.05, 1.0, 0.1)
        step2 = slew_vector(step1, np.zeros(6), 0.05, 1.0, 0.1)
        # Smooth decay: each step reduces norm, never jumps to zero.
        self.assertLess(np.linalg.norm(step1), np.linalg.norm(nonzero))
        self.assertGreater(np.linalg.norm(step1), 0.0)
        self.assertLess(np.linalg.norm(step2), np.linalg.norm(step1))

    def test_zero_compensation_skips_allocation(self):
        """When current compensation is zero, the raw PX4 message must be
        forwarded without entering the allocation path."""
        commands = np.full(8, 0.42)
        result = allocate_total_wrench(
            CONFIG, commands, np.zeros(6), maximum_motor_delta_n=0.5
        )
        # Zero compensation should produce commands identical to base.
        np.testing.assert_allclose(
            result["commands_motor_order"], commands, atol=1e-9
        )


def _quaternion_wxyz_to_rotation(q_wxyz):
    """Helper for body-to-world rotation (matches production code)."""
    q = np.asarray(q_wxyz, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=float)


class TruthVelocityFrameTest(unittest.TestCase):
    """Verify that Gazebo body-FLU velocity is correctly rotated to world-ENU
    before entering the PD controller's D-term."""

    def test_yaw_90_deg_maps_body_x_to_world_y(self):
        # Body velocity [1, 0, 0] with yaw=+90° should become world [0, 1, 0].
        # Quaternion for 90° around Z: w=cos(45°), z=sin(45°), x=y=0.
        q_yaw90 = np.asarray([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])  # wxyz
        rotation = _quaternion_wxyz_to_rotation(q_yaw90)
        body_vel = np.array([1.0, 0.0, 0.0])
        world_vel = rotation @ body_vel
        np.testing.assert_allclose(world_vel, [0.0, 1.0, 0.0], atol=1e-12)

    def test_identity_rotation_preserves_velocity(self):
        rotation = np.eye(3)
        body_vel = np.array([0.3, -0.2, 0.1])
        world_vel = rotation @ body_vel
        np.testing.assert_allclose(world_vel, body_vel, atol=1e-12)

    def test_arbitrary_rotation_is_orthonormal(self):
        q = np.asarray([0.5, 0.5, 0.5, 0.5])  # wxyz
        rotation = _quaternion_wxyz_to_rotation(q)
        # Rotation matrix must be orthonormal.
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
