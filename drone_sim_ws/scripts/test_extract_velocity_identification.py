#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


PATH = Path(__file__).with_name("extract_velocity_identification.py")
SPEC = importlib.util.spec_from_file_location("extract_velocity_identification", PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VelocityEvidenceExtractorTest(unittest.TestCase):
    @staticmethod
    def _complete_stage(target: float, suffix: str) -> dict:
        return {
            "samples": 40,
            f"target_{suffix}": target,
            f"actual_peak_directed_{suffix}": abs(target) * 1.04,
            "rise_time_s": 0.5,
            "overshoot_fraction": 0.04,
            "settling_time_s": 0.9,
            f"steady_error_{suffix}": 0.01,
        }

    def test_existing_1hz_log_is_chain_evidence_not_tuning_evidence(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        report = MODULE.extract(
            workspace / "analysis/velocity_full_latest_wrench_scurve.log", 1.0
        )
        self.assertTrue(report["required_markers_present"])
        self.assertFalse(report["ready_for_velocity_tuning"])
        self.assertEqual(report["result"], "VELOCITY_IDENTIFICATION_EVIDENCE_INCOMPLETE")

    def test_missing_landing_marker_is_rejected(self) -> None:
        metrics = {
            "phase_horizontal_response": {},
            "phase_vertical_response": {},
            "phase_yaw_response": {},
        }
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "run.log"
            log.write_text(
                MODULE.MARKER + json.dumps(metrics) + "\nDDS_VELOCITY_WASD_PASS\n",
                encoding="utf-8",
            )
            report = MODULE.extract(log, 10.0)
            self.assertIn("LANDING_DISARMED_CONFIRMED", report["missing_markers"])
            self.assertFalse(report["ready_for_velocity_tuning"])

    def test_complete_hashed_10hz_log_is_tuning_ready(self) -> None:
        horizontal = {
            name: self._complete_stage(0.4, "m_s")
            for name in ("W_0", "S_1", "A_2", "D_3")
        }
        vertical = {
            "R_4": self._complete_stage(-0.15, "m_s"),
            "F_5": self._complete_stage(0.15, "m_s"),
        }
        yaw = {
            "Q_6": self._complete_stage(-15.0, "deg_s"),
            "E_7": self._complete_stage(15.0, "deg_s"),
        }
        metrics = {
            "phase_horizontal_response": horizontal,
            "phase_vertical_response": vertical,
            "phase_yaw_response": yaw,
            "phase_quality": {
                "H_9": {"tail_1s_max_truth_roll_pitch_deg": 1.2}
            },
            "horizontal_speed_overshoot_fraction": 0.08,
            "vertical_speed_overshoot_fraction": 0.06,
            "yaw_rate_overshoot_fraction": 0.05,
            "motor_saturation_fraction": 0.0,
        }
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "run.log"
            log.write_text(
                MODULE.MARKER + json.dumps(metrics) + "\n"
                "DDS_VELOCITY_WASD_PASS\nLANDING_DISARMED_CONFIRMED\n",
                encoding="utf-8",
            )
            report = MODULE.extract(log, 10.0)
        self.assertTrue(report["ready_for_velocity_tuning"])
        self.assertEqual(report["result"], "VELOCITY_IDENTIFICATION_EVIDENCE_PASS")
        self.assertEqual(len(report["source_log_sha256"]), 64)
        self.assertTrue(all(report["acceptance"].values()))
        self.assertTrue(report["overshoot_consistency_pass"])

    def test_understated_aggregate_overshoot_is_rejected(self) -> None:
        horizontal = {
            name: self._complete_stage(0.4, "m_s")
            for name in ("W_0", "S_1", "A_2", "D_3")
        }
        horizontal["W_0"]["overshoot_fraction"] = 0.20
        vertical = {
            "R_4": self._complete_stage(-0.15, "m_s"),
            "F_5": self._complete_stage(0.15, "m_s"),
        }
        yaw = {
            "Q_6": self._complete_stage(-15.0, "deg_s"),
            "E_7": self._complete_stage(15.0, "deg_s"),
        }
        metrics = {
            "phase_horizontal_response": horizontal,
            "phase_vertical_response": vertical,
            "phase_yaw_response": yaw,
            "phase_quality": {
                "H_9": {"tail_1s_max_truth_roll_pitch_deg": 1.0}
            },
            # This deliberately contradicts W_0's measured 20% stage value.
            "horizontal_speed_overshoot_fraction": 0.05,
            "vertical_speed_overshoot_fraction": 0.05,
            "yaw_rate_overshoot_fraction": 0.05,
            "motor_saturation_fraction": 0.0,
        }
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "understated.log"
            log.write_text(
                MODULE.MARKER + json.dumps(metrics) + "\n"
                "DDS_VELOCITY_WASD_PASS\nLANDING_DISARMED_CONFIRMED\n",
                encoding="utf-8",
            )
            report = MODULE.extract(log, 10.0)
        self.assertFalse(report["overshoot_consistency_pass"])
        self.assertFalse(report["aggregate_overshoot_not_understated"]["horizontal"])
        self.assertFalse(report["ready_for_velocity_tuning"])


if __name__ == "__main__":
    unittest.main()
