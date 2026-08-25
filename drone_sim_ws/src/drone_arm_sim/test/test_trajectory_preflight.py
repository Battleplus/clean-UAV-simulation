from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from drone_arm_sim.trajectory_preflight import (
    TrajectoryPreflight,
    quintic_joint_samples,
)


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
REFERENCE = PACKAGE / "config/so101_motion_reference_4kg.json"
CONFIG = PACKAGE / "config/my_drone_v3_cad_debug_4kg.json"


class TrajectoryPreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.preflight = TrajectoryPreflight(URDF, REFERENCE, CONFIG)
        cls.home = cls.preflight.dynamics.home_positions
        forward_values = cls.preflight.reference["presets"]["flight_straight_forward"]
        cls.forward = dict(zip(cls.preflight.names, forward_values, strict=True))

    def test_validated_ninety_second_forward_trajectory_is_accepted(self):
        report = self.preflight.adapt_quintic(
            self.home,
            self.forward,
            90.0,
            allow_distance_scaling=False,
        )
        self.assertTrue(report["accepted"])
        self.assertEqual(report["selected"]["distance_scale"], 1.0)
        self.assertLessEqual(report["selected"]["time_scale"], 1.5)
        evaluation = report["selected"]["evaluation"]
        self.assertLessEqual(
            evaluation["maximum_allocation_residual_norm"],
            self.preflight.allocation_residual_limit,
        )
        self.assertGreaterEqual(
            evaluation["minimum_physical_motor_headroom_n"],
            self.preflight.minimum_motor_headroom_n,
        )
        self.assertIn("maximum_joint_velocity_rad_s", evaluation)
        self.assertIn("maximum_joint_acceleration_rad_s2", evaluation)
        self.assertIn("maximum_joint_jerk_rad_s3", evaluation)

    def test_preflight_includes_all_rotor_sweeps_without_global_folded_override(self):
        rotor_boxes = [
            box
            for box in self.preflight.boxes
            if box.source == "rotor_visual_swept_volume"
        ]
        self.assertEqual(len(rotor_boxes), 8)
        folded_pair = ("upper_arm_link", "wrist_link")
        self.assertNotIn(folded_pair, self.preflight.ignored_collision_pairs)
        self.assertIn(
            folded_pair, self.preflight.retracted_anchor_collision_overrides
        )
        self.assertLessEqual(self.preflight.retracted_override_tolerance_rad, 0.20)

    def test_fast_request_is_slowed_instead_of_published_unchanged(self):
        report = self.preflight.adapt_quintic(
            self.home,
            self.forward,
            5.0,
            allow_distance_scaling=False,
        )
        self.assertTrue(report["accepted"])
        self.assertGreater(report["selected"]["time_scale"], 1.0)
        self.assertIn(report["decision"], ("slowed", "shortened_and_slowed"))

    def test_return_to_home_is_never_distance_shortened(self):
        report = self.preflight.adapt_quintic(
            self.forward,
            self.home,
            5.0,
            # Even a mistaken caller request may not shorten a retraction.
            allow_distance_scaling=True,
        )
        self.assertTrue(report["is_retraction"])
        self.assertFalse(report["distance_scaling_allowed"])
        if report["accepted"]:
            self.assertEqual(report["selected"]["distance_scale"], 1.0)
        self.assertTrue(all(
            item["distance_scale"] == 1.0 for item in report["attempts"]
        ))

    def test_adaptation_order_exhausts_full_distance_before_shortening(self):
        rejected = {"accepted": False, "failure_counts": {"synthetic": 1}}
        with patch.object(self.preflight, "evaluate", return_value=rejected):
            report = self.preflight.adapt_quintic(
                self.home,
                self.forward,
                1.0,
                allow_distance_scaling=True,
                time_scales=(4.0, 2.0),
                distance_scales=(0.5, 0.9),
                sample_count=5,
            )
        self.assertFalse(report["accepted"])
        observed = [
            (item["distance_scale"], item["time_scale"])
            for item in report["attempts"]
        ]
        self.assertEqual(
            observed,
            [
                (1.0, 1.0), (1.0, 2.0), (1.0, 4.0),
                (0.9, 1.0), (0.9, 2.0), (0.9, 4.0),
                (0.5, 1.0), (0.5, 2.0), (0.5, 4.0),
            ],
        )

    def test_planner_can_skip_a_geometrically_rejected_full_distance(self):
        rejected = {"accepted": False, "failure_counts": {"synthetic": 1}}
        with patch.object(self.preflight, "evaluate", return_value=rejected):
            report = self.preflight.adapt_quintic(
                self.home,
                self.forward,
                1.0,
                allow_distance_scaling=True,
                time_scales=(1.0,),
                distance_scales=(0.8, 0.6),
                sample_count=5,
                try_full_distance=False,
            )
        self.assertFalse(report["accepted"])
        self.assertEqual(
            [item["distance_scale"] for item in report["attempts"]],
            [0.8, 0.6],
        )

    def test_shortened_decision_returns_the_exact_evaluated_target(self):
        q0 = np.asarray([self.home[name] for name in self.preflight.names])
        q1 = np.asarray([self.forward[name] for name in self.preflight.names])

        def accept_only_eighty_percent(samples):
            endpoint = np.asarray(
                [samples[-1]["positions"][name] for name in self.preflight.names]
            )
            scale = np.linalg.norm(endpoint - q0) / np.linalg.norm(q1 - q0)
            return {"accepted": abs(scale - 0.8) < 1.0e-9, "failure_counts": {}}

        with patch.object(self.preflight, "evaluate", side_effect=accept_only_eighty_percent):
            report = self.preflight.adapt_quintic(
                self.home,
                self.forward,
                1.0,
                allow_distance_scaling=True,
                time_scales=(1.0,),
                distance_scales=(0.8,),
                sample_count=5,
            )
        self.assertTrue(report["accepted"])
        self.assertEqual(report["decision"], "shortened")
        selected = report["selected"]
        self.assertEqual(selected["distance_scale"], 0.8)
        selected_q = np.asarray(
            [selected["selected_target"][name] for name in self.preflight.names]
        )
        np.testing.assert_allclose(selected_q, q0 + 0.8 * (q1 - q0))

    def test_quintic_duration_scaling_reduces_velocity_acceleration_and_jerk(self):
        fast = quintic_joint_samples(
            self.preflight.names, self.home, self.forward, 2.0, sample_count=101
        )
        slow = quintic_joint_samples(
            self.preflight.names, self.home, self.forward, 4.0, sample_count=101
        )

        def peak(samples, field):
            return max(
                abs(float(value))
                for sample in samples
                for value in sample[field].values()
            )

        self.assertAlmostEqual(peak(slow, "velocities"), peak(fast, "velocities") / 2.0)
        self.assertAlmostEqual(
            peak(slow, "accelerations"), peak(fast, "accelerations") / 4.0
        )
        self.assertAlmostEqual(peak(slow, "jerks"), peak(fast, "jerks") / 8.0)

    def test_invalid_adaptation_scales_are_rejected(self):
        with self.assertRaises(ValueError):
            self.preflight.adapt_quintic(
                self.home,
                self.forward,
                1.0,
                allow_distance_scaling=False,
                time_scales=(0.5,),
            )
        with self.assertRaises(ValueError):
            self.preflight.adapt_quintic(
                self.home,
                self.forward,
                1.0,
                allow_distance_scaling=True,
                distance_scales=(1.1,),
            )
        with self.assertRaises(ValueError):
            self.preflight.adapt_quintic(
                self.forward,
                self.home,
                1.0,
                allow_distance_scaling=True,
                try_full_distance=False,
            )


if __name__ == "__main__":
    unittest.main()
