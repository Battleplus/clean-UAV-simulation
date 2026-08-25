import json
from pathlib import Path
import unittest

import numpy as np

from drone_arm_sim.workspace_envelope import (
    OrientedBox,
    _gripper_state,
    _radial_band,
    _rotor_swept_volume_proxies,
    _vertical_direction,
    collision_pairs,
    oriented_boxes_intersect,
    scan_workspace,
)


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
REFERENCE = PACKAGE / "config/so101_motion_reference_4kg.json"
CONFIG = PACKAGE / "config/my_drone_v3_cad_debug_4kg.json"


class WorkspaceEnvelopeTest(unittest.TestCase):
    def test_oriented_box_separating_axis(self):
        identity = np.eye(4)
        separated = np.eye(4)
        separated[0, 3] = 3.0
        rotated = np.eye(4)
        angle = np.pi / 4.0
        rotated[:3, :3] = np.asarray(
            [[np.cos(angle), -np.sin(angle), 0.0],
             [np.sin(angle), np.cos(angle), 0.0],
             [0.0, 0.0, 1.0]]
        )
        rotated[0, 3] = 0.8
        half = np.ones(3) * 0.5
        self.assertFalse(oriented_boxes_intersect(identity, half, separated, half))
        self.assertTrue(oriented_boxes_intersect(identity, half, rotated, half))

    def test_quick_scan_covers_validated_forward_anchor(self):
        counts = {
            "shoulder_pan": 2,
            "shoulder_lift": 2,
            "elbow_flex": 2,
            "wrist_flex": 2,
            "wrist_roll": 2,
            "gripper": 2,
        }
        report = scan_workspace(URDF, REFERENCE, CONFIG, grid_counts=counts)
        self.assertEqual(report["schema"], 2)
        self.assertNotIn("samples", report)
        self.assertIn("maximum_radius_by_direction", report["boundary_samples"])
        forward = report["anchors"]["flight_straight_forward"]
        self.assertTrue(forward["flight_allowed"], forward["rejection_reasons"])
        self.assertLess(forward["allocation_residual_norm"], 1.0e-6)
        self.assertGreater(forward["remaining_overlay_delta_headroom_n"], 0.05)
        self.assertGreater(forward["minimum_joint_effort_margin_nm"], 0.0)
        self.assertEqual(forward["positions_rad"]["shoulder_pan"], 0.0)
        self.assertEqual(report["collision_proxy"]["rotor_swept_volume_count"], 8)
        self.assertEqual(len(report["inputs"]["sha256"]["urdf"]), 64)
        self.assertNotIn(
            ["upper_arm_link", "wrist_link"],
            report["collision_proxy"]["ignored_link_pairs"],
        )
        self.assertIn(
            ["upper_arm_link", "wrist_link"],
            report["anchors"]["retracted"]["anchor_collision_override_pairs"],
        )
        self.assertEqual(
            report["anchors"]["flight_straight_forward"][
                "anchor_collision_override_pairs"
            ],
            [],
        )

    def test_retracted_anchor_has_zero_incremental_gravity_torque(self):
        counts = {name: 2 for name in (
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper",
        )}
        report = scan_workspace(URDF, REFERENCE, CONFIG, grid_counts=counts)
        retracted = report["anchors"]["retracted"]
        self.assertTrue(retracted["flight_allowed"])
        self.assertAlmostEqual(retracted["gravity_torque_norm_nm"], 0.0, places=10)

    def test_direction_distance_and_gripper_classification(self):
        self.assertEqual(_vertical_direction(0.04), "up")
        self.assertEqual(_vertical_direction(-0.04), "down")
        self.assertEqual(_vertical_direction(0.01), "level")
        self.assertEqual(_radial_band(0.10), "near")
        self.assertEqual(_radial_band(0.20), "middle")
        self.assertEqual(_radial_band(0.30), "far")
        self.assertEqual(_gripper_state(0.0, 0.0, 1.0), "closed")
        self.assertEqual(_gripper_state(0.5, 0.0, 1.0), "middle")
        self.assertEqual(_gripper_state(1.0, 0.0, 1.0), "open")

    def test_rotor_swept_volume_is_derived_from_all_eight_stls(self):
        proxies = _rotor_swept_volume_proxies(URDF)
        self.assertEqual(len(proxies), 8)
        self.assertEqual({item.link for item in proxies}, {
            f"rotor_{index}_link" for index in range(1, 9)
        })
        for item in proxies:
            self.assertEqual(item.source, "rotor_visual_swept_volume")
            # CAD propeller radius is about 64 mm and the configured proxy
            # includes a 5 mm clearance around the whole swept disc.
            self.assertGreater(item.half_extent_m[0], 0.068)
            self.assertLess(item.half_extent_m[0], 0.070)
            self.assertAlmostEqual(item.half_extent_m[0], item.half_extent_m[1])

    def test_report_exposes_cross_product_coverage_evidence(self):
        counts = {
            "shoulder_pan": 3,
            "shoulder_lift": 3,
            "elbow_flex": 3,
            "wrist_flex": 3,
            "wrist_roll": 3,
            "gripper": 2,
        }
        report = scan_workspace(URDF, REFERENCE, CONFIG, grid_counts=counts)
        coverage = report["coverage"]
        self.assertEqual(set(coverage["direction_vertical_matrix"]), {
            "front", "front_left", "left", "rear_left", "rear",
            "rear_right", "right", "front_right",
        })
        self.assertEqual(set(coverage["direction_radial_matrix"]["front"]), {
            "near", "middle", "far",
        })
        self.assertEqual(set(coverage["direction_gripper_matrix"]["front"]), {
            "closed", "middle", "open",
        })
        self.assertIn("coverage_checks", report["summary"])
        self.assertFalse(report["model_assumptions"]["formal_physical_release_ready"])


if __name__ == "__main__":
    unittest.main()
