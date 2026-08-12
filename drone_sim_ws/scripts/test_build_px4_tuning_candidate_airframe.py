#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


PATH = Path(__file__).with_name("build_px4_tuning_candidate_airframe.py")
SPEC = importlib.util.spec_from_file_location("build_px4_tuning_candidate_airframe", PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CandidateAirframeBuilderTest(unittest.TestCase):
    def test_bounded_debug_candidate_is_written_without_touching_source(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "4027_debug_4kg"
            decision = root / "decision.json"
            output = root / "4028_debug_4kg_candidate"
            source.write_text("param set-default MC_ROLL_P 3.00\n", encoding="utf-8")
            original = source.read_bytes()
            decision.write_text(json.dumps({
                "decision": "BOUNDED_CANDIDATE_REQUIRES_RETEST",
                "parameter_changes": [{
                    "parameter": "MC_ROLL_P",
                    "old": 3.0,
                    "candidate": 2.85,
                    "relative_change": -0.05,
                    "status": "EXPERIMENTAL_NOT_APPLIED",
                }],
            }), encoding="utf-8")
            result = MODULE.build(source, decision, output)
            self.assertEqual(source.read_bytes(), original)
            self.assertIn("MC_ROLL_P 2.85", output.read_text(encoding="utf-8"))
            self.assertTrue(result["formal_7p735_untouched"])

    def test_formal_airframe_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "4026_gz_my_drone_octorotor_7p735"
            decision = root / "decision.json"
            output = root / "candidate"
            source.write_text("param set-default MC_ROLL_P 3.0\n", encoding="utf-8")
            decision.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "only accepts"):
                MODULE.build(source, decision, output)

    def test_incomplete_decision_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "4027_debug_4kg"
            decision = root / "decision.json"
            output = root / "4028_debug_4kg_candidate"
            source.write_text("param set-default MC_ROLL_P 3.0\n", encoding="utf-8")
            decision.write_text(json.dumps({
                "decision": "HOLD_INCOMPLETE_IDENTIFICATION",
                "parameter_changes": [],
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "does not authorize"):
                MODULE.build(source, decision, output)


if __name__ == "__main__":
    unittest.main()
