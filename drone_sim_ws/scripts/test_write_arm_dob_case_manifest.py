from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile

import pytest


SCRIPT = Path(__file__).with_name("write_arm_dob_case_manifest.py")
SPEC = importlib.util.spec_from_file_location("write_arm_dob_case_manifest", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_manifest_hashes_every_required_file_and_only_switch_differs():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        files = {}
        for name in (
            "config", "urdf", "world", "airframe", "backend_launcher", "flight_driver",
            "observer_source", "motor_model_source", "coupling_monitor_source",
            "px4_binary",
        ):
            path = root / name
            path.write_text(f"{name}\n", encoding="utf-8")
            files[name] = path
        off = MODULE.build_manifest(
            label="off", observer_enabled=False, observer_gain=0.25, files=files
        )
        on = MODULE.build_manifest(
            label="on", observer_enabled=True, observer_gain=0.25, files=files
        )
    assert off["common"] == on["common"]
    assert off["observer_enabled"] is False
    assert on["observer_enabled"] is True
    assert all(len(item["sha256"]) == 64 for item in off["common"]["files"].values())


def test_manifest_rejects_out_of_range_gain():
    with pytest.raises(ValueError, match="observer gain"):
        MODULE.build_manifest(
            label="bad", observer_enabled=True, observer_gain=0.0, files={}
        )
