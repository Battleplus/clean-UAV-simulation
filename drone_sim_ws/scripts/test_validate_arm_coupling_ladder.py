#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("validate_arm_coupling_ladder.py")
SPEC = importlib.util.spec_from_file_location("validate_arm_coupling_ladder", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ArmCouplingLadderValidatorTest(unittest.TestCase):
    def test_current_report_is_backed_by_raw_logs(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        report = workspace / "analysis" / "arm_coupling_ladder_4kg_20260811.json"
        result = MODULE.validate(workspace, report)
        self.assertEqual(result["result"], "ARM_COUPLING_LADDER_EVIDENCE_PASS")
        self.assertEqual(result["stage_count"], 6)
        self.assertEqual(result["accepted_run_count"], 7)
        self.assertEqual(
            result["full_tensor_model_evidence"]["result"],
            "ARM_COUPLING_TENSOR_REPORT_PASS",
        )

    def test_missing_landing_marker_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "bad.log"
            log.write_text(
                "ARM_FLIGHT_METRICS horizontal_drift_m=0 altitude_span_m=0 "
                "max_com_shift_m=0 max_inertia_diag_change_kg_m2=0 "
                "max_arm_force_n=0 max_arm_torque_nm=0 "
                "motor_saturation_samples=0/1 max_truth_tilt_deg=0\n"
                "DDS_ARM_FLIGHT_PASS\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "landing/disarm"):
                MODULE.parse_metrics(log)

    def test_stale_tensor_hash_is_rejected(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        source_report = (
            workspace / "analysis/arm_coupling_full_tensor_4kg_modelcheck_20260812.json"
        )
        source_validation = source_report.with_suffix(".validation.json")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "tensor.json"
            validation = root / "tensor.validation.json"
            shutil.copyfile(source_report, report)
            shutil.copyfile(source_validation, validation)
            evidence = {
                "report": "tensor.json",
                "validation": "tensor.validation.json",
                "report_sha256": "0" * 64,
            }
            with self.assertRaisesRegex(ValueError, "stale"):
                MODULE.validate_tensor_evidence(root, evidence)


if __name__ == "__main__":
    unittest.main()
