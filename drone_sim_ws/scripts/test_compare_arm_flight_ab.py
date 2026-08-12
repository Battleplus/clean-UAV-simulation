from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import compare_arm_flight_ab as comparison


def flight_log(
    *,
    drift: float,
    altitude: float,
    max_tilt: float,
    rms_tilt: float,
    saturation: int = 0,
) -> str:
    return "\n".join(
        (
            "ARM_FLIGHT_HOVER_READY",
            "ARM_PRESET_REACHED preset=demo_extended max_error=0.010000",
            "ARM_PRESET_REACHED preset=retracted max_error=0.010000",
            (
                "ARM_FLIGHT_METRICS "
                f"horizontal_drift_m={drift:.3f} "
                f"altitude_span_m={altitude:.3f} "
                "max_com_shift_m=0.010000 "
                "max_inertia_diag_change_kg_m2=0.001000000 "
                "max_arm_force_n=0.200000 "
                "max_arm_torque_nm=0.100 "
                f"motor_saturation_samples={saturation}/100 "
                f"motor_saturation_rate={saturation / 100.0:.6f} "
                "rated_motor_output=1.000 "
                f"max_truth_tilt_deg={max_tilt:.3f} "
                f"rms_truth_tilt_deg={rms_tilt:.3f} "
                "samples=100"
            ),
            "DDS_ARM_FLIGHT_PASS",
            "",
        )
    )


class CompareArmFlightAbTest(unittest.TestCase):
    @staticmethod
    def _case_manifest(root: Path, enabled: bool, *, gain: float = 0.25) -> Path:
        source = root / "config.json"
        if not source.exists():
            source.write_text("{}\n", encoding="utf-8")
        payload = source.read_bytes()
        common = {
            "observer_gain": gain,
            "files": {
                "config": {
                    "path": str(source.resolve()),
                    "size_bytes": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            },
        }
        path = root / ("on.json" if enabled else "off.json")
        path.write_text(
            json.dumps(
                {
                    "schema": 1,
                    "observer_enabled": enabled,
                    "common": common,
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_strict_parser_accepts_complete_zero_saturation_log(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.log"
            path.write_text(
                flight_log(drift=0.10, altitude=0.20, max_tilt=2.0, rms_tilt=1.0),
                encoding="utf-8",
            )
            result = comparison.parse_log(
                path, ("demo_extended", "retracted"), 0.15, 0.30, 3.0, 0.50
            )
        self.assertTrue(result["strict_diagnostics_present"])
        self.assertTrue(result["strict_diagnostics_ok"])
        self.assertTrue(result["within_current_gate"])
        self.assertEqual(result["motor_saturation_samples"], 0)
        self.assertEqual(len(result["log_sha256"]), 64)
        self.assertGreater(result["log_size_bytes"], 0)

    def test_saturation_rejects_run_even_when_position_gates_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.log"
            path.write_text(
                flight_log(
                    drift=0.10,
                    altitude=0.20,
                    max_tilt=2.0,
                    rms_tilt=1.0,
                    saturation=1,
                ),
                encoding="utf-8",
            )
            result = comparison.parse_log(
                path, ("demo_extended", "retracted"), 0.15, 0.30, 3.0, 0.50
            )
        self.assertFalse(result["strict_diagnostics_ok"])
        self.assertFalse(result["within_current_gate"])

    def test_require_improvement_covers_drift_altitude_and_both_tilt_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            off = root / "off.log"
            on = root / "on.log"
            off.write_text(
                flight_log(drift=0.10, altitude=0.20, max_tilt=2.0, rms_tilt=1.2),
                encoding="utf-8",
            )
            on.write_text(
                flight_log(drift=0.08, altitude=0.15, max_tilt=1.8, rms_tilt=1.0),
                encoding="utf-8",
            )
            arguments = [
                "compare_arm_flight_ab.py",
                "--feedforward-off",
                str(off),
                "--feedforward-on",
                str(on),
                "--required-presets",
                "demo_extended",
                "retracted",
                "--horizontal-gate",
                "0.15",
                "--altitude-gate",
                "0.30",
                "--require-improvement",
            ]
            with patch.object(sys, "argv", arguments):
                self.assertEqual(comparison.main(), 0)

            on.write_text(
                flight_log(drift=0.08, altitude=0.15, max_tilt=1.8, rms_tilt=1.3),
                encoding="utf-8",
            )
            with patch.object(sys, "argv", arguments):
                self.assertEqual(comparison.main(), 3)

    def test_candidate_runtime_death_invalidates_an_apparently_good_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            off = root / "off.log"
            on = root / "on.log"
            runtime = root / "on_gazebo.log"
            off.write_text(
                flight_log(drift=0.10, altitude=0.20, max_tilt=2.0, rms_tilt=1.2),
                encoding="utf-8",
            )
            on.write_text(
                flight_log(drift=0.08, altitude=0.15, max_tilt=1.8, rms_tilt=1.0),
                encoding="utf-8",
            )
            runtime.write_text(
                "arm_disturbance_observer: error: unrecognized arguments: --ros-args\n"
                "[ERROR] [arm_disturbance_observer-10]: process has died exit code 2\n",
                encoding="utf-8",
            )
            arguments = [
                "compare_arm_flight_ab.py",
                "--feedforward-off", str(off),
                "--feedforward-on", str(on),
                "--candidate-runtime-log", str(runtime),
                "--required-presets", "demo_extended", "retracted",
                "--horizontal-gate", "0.15",
                "--altitude-gate", "0.30",
                "--require-improvement",
            ]
            with patch.object(sys, "argv", arguments):
                self.assertEqual(comparison.main(), 3)
            evidence = comparison.parse_candidate_runtime(runtime, "ARM_DOB_STATE")
            self.assertFalse(evidence["valid"])
            self.assertTrue(evidence["observer_process_died"])
            self.assertTrue(evidence["cli_argument_failure"])

    def test_candidate_runtime_requires_active_nonzero_observer_output(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / "on_gazebo.log"
            inactive = {
                "active": False,
                "estimated_disturbance_torque_frd_nm": [0.0, 0.0, 0.0],
            }
            active = {
                "active": True,
                "estimated_disturbance_torque_frd_nm": [0.01, -0.02, 0.0],
            }
            runtime.write_text(
                "ARM_DOB_STATE " + json.dumps(inactive) + "\n"
                "ARM_DOB_STATE " + json.dumps(active) + "\n",
                encoding="utf-8",
            )
            evidence = comparison.parse_candidate_runtime(runtime, "ARM_DOB_STATE")
        self.assertTrue(evidence["valid"])
        self.assertEqual(evidence["active_marker_count"], 1)
        self.assertEqual(len(evidence["log_sha256"]), 64)
        self.assertGreater(evidence["log_size_bytes"], 0)
        self.assertAlmostEqual(
            evidence["maximum_estimated_torque_nm"],
            (0.01 ** 2 + 0.02 ** 2) ** 0.5,
        )

    def test_pair_manifests_require_only_observer_enable_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            off = self._case_manifest(root, False)
            on = self._case_manifest(root, True)
            evidence = comparison.compare_case_manifests(off, on)
        self.assertTrue(evidence["valid"])
        self.assertTrue(evidence["only_observer_enable_differs"])
        self.assertEqual(len(evidence["manifests"]), 2)

    def test_pair_manifests_reject_common_configuration_change(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            off = self._case_manifest(root, False, gain=0.25)
            on = self._case_manifest(root, True, gain=0.30)
            evidence = comparison.compare_case_manifests(off, on)
        self.assertFalse(evidence["valid"])
        self.assertIn(
            "OFF and ON common configuration manifests differ",
            evidence["errors"],
        )


if __name__ == "__main__":
    unittest.main()
