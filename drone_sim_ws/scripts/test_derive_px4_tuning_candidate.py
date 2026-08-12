#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


PATH = Path(__file__).with_name("derive_px4_tuning_candidate.py")
SPEC = importlib.util.spec_from_file_location("derive_px4_tuning_candidate", PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def complete_axis(overshoot: float) -> dict[str, object]:
    return {
        "step_evaluable": True,
        "target_value": 1.0,
        "actual_peak": 1.0 + overshoot,
        "rise_time_s": 0.2,
        "overshoot_fraction": overshoot,
        "settling_time_s": 0.5,
        "steady_error": 0.01,
    }


class TuningCandidateTest(unittest.TestCase):
    def test_current_v2_report_holds_incomplete_rate_layer(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        import json

        report = json.loads(
            (workspace / "analysis/inner_loop_identification_4kg_20260812_v2_loops.json").read_text()
        )
        parameters = MODULE.parse_airframe_parameters(
            workspace / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
        )
        result = MODULE.evaluate(report, parameters)
        self.assertEqual(result["decision"], "HOLD_INCOMPLETE_IDENTIFICATION")
        self.assertEqual(result["active_layer"], "body_rate")
        self.assertEqual(result["parameter_changes"], [])

    def test_outer_layer_cannot_tune_before_rate_layer_passes(self) -> None:
        report = {
            "motors": {"saturation_fraction": 0.0},
            "loops": {"body_rate_rad_s": {"roll": complete_axis(0.11)}},
        }
        result = MODULE.evaluate(report, {"MC_ROLLRATE_P": 0.1})
        self.assertEqual(result["decision"], "HOLD_INCOMPLETE_IDENTIFICATION")
        self.assertEqual(result["active_layer"], "body_rate")

    def test_complete_rate_overshoot_yields_bounded_candidate(self) -> None:
        report = {
            "motors": {"saturation_fraction": 0.0},
            "loops": {
                "body_rate_rad_s": {
                    "roll": complete_axis(0.14),
                    "pitch": complete_axis(0.05),
                    "yaw": complete_axis(0.05),
                }
            },
        }
        result = MODULE.evaluate(
            report,
            {"MC_ROLLRATE_P": 0.1, "MC_PITCHRATE_P": 0.1, "MC_YAWRATE_P": 0.12},
        )
        self.assertEqual(result["decision"], "BOUNDED_CANDIDATE_REQUIRES_RETEST")
        self.assertEqual(result["active_layer"], "body_rate")
        self.assertEqual(len(result["parameter_changes"]), 1)
        change = result["parameter_changes"][0]
        self.assertEqual(change["parameter"], "MC_ROLLRATE_P")
        self.assertGreaterEqual(change["relative_change"], -0.05)
        self.assertLess(change["candidate"], change["old"])

    def test_complete_inner_layers_still_require_raw_velocity_evidence(self) -> None:
        complete = {
            axis: complete_axis(0.05) for axis in ("roll", "pitch", "yaw")
        }
        report = {
            "motors": {"saturation_fraction": 0.0},
            "loops": {
                "body_rate_rad_s": complete,
                "attitude_rad": complete,
            },
        }
        result = MODULE.evaluate(report, {}, velocity_evidence=None)
        self.assertEqual(result["decision"], "HOLD_INCOMPLETE_IDENTIFICATION")
        self.assertEqual(result["active_layer"], "velocity")
        self.assertEqual(result["passed_layers"], ["body_rate", "attitude"])


if __name__ == "__main__":
    unittest.main()
