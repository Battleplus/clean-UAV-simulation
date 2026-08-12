#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("validate_motor_thrust_evidence.py")
SPEC = importlib.util.spec_from_file_location("validate_motor_thrust_evidence", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MotorEvidenceReferenceTest(unittest.TestCase):
    def test_missing_local_reference_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            evidence_path = Path(directory) / "evidence.json"
            evidence = {"motors": [{"motor": 4, "evidence_refs": ["motor4.jpg"]}]}
            errors, manifest = MODULE.validate_reference_files(evidence, evidence_path)
            self.assertEqual(manifest, [])
            self.assertEqual(
                errors,
                ["motor 4: evidence file does not exist: motor4.jpg"],
            )

    def test_existing_reference_is_hashed_once(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            evidence_path = root / "evidence.json"
            photo = root / "motor4.jpg"
            photo.write_bytes(b"auditable-propeller-photo")
            evidence = {
                "motors": [
                    {"motor": 4, "evidence_refs": ["motor4.jpg"]},
                    {"motor": 5, "evidence_refs": ["motor4.jpg"]},
                ]
            }
            errors, manifest = MODULE.validate_reference_files(evidence, evidence_path)
            self.assertEqual(errors, [])
            self.assertEqual(len(manifest), 1)
            self.assertEqual(manifest[0]["path"], "motor4.jpg")
            self.assertEqual(
                manifest[0]["sha256"],
                hashlib.sha256(b"auditable-propeller-photo").hexdigest(),
            )


if __name__ == "__main__":
    unittest.main()
