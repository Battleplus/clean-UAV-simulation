from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PATH = Path(__file__).with_name("validate_arm_coupling_tensor_report.py")
SPEC = importlib.util.spec_from_file_location("validate_arm_coupling_tensor_report", PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ArmCouplingTensorReportTest(unittest.TestCase):
    def test_current_4kg_report_passes(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        result = MODULE.validate(
            workspace / "analysis/arm_coupling_full_tensor_4kg_modelcheck_20260812.json",
            4.0,
            0.05,
        )
        self.assertEqual(result["result"], "ARM_COUPLING_TENSOR_REPORT_PASS")

    def test_mislabeled_mass_is_rejected(self) -> None:
        workspace = Path(__file__).resolve().parents[1]
        source = workspace / "analysis/arm_coupling_full_tensor_4kg_modelcheck_20260812.json"
        payload = json.loads(source.read_text(encoding="utf-8"))
        payload["model_mass_without_payload_kg"] = 7.735
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "base mass"):
                MODULE.validate(path, 4.0, 0.05)


if __name__ == "__main__":
    unittest.main()
