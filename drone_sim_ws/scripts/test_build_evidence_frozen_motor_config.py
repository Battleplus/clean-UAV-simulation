from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).with_name("build_evidence_frozen_motor_config.py")
SPEC = importlib.util.spec_from_file_location("build_evidence_frozen_motor_config", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
WORKSPACE = Path(__file__).resolve().parents[1]


def fixtures():
    source = json.loads(
        (WORKSPACE / "src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json")
        .read_text(encoding="utf-8")
    )
    frozen = json.loads(
        (WORKSPACE / "analysis/cad_direct/motor_physical_freeze_status.json")
        .read_text(encoding="utf-8")
    )
    evidence = {
        "motors": [
            {
                "motor": motor,
                "thrust_sign_along_cad_propeller_ray": -1 if motor in {3, 6, 7, 8} else 1,
                "prop_type": "REVERSE" if motor in {4, 5, 7, 8} else "NORMAL",
                "propeller_part_number": f"P{motor}",
                "evidence_type": "manufacturer",
            }
            for motor in range(1, 9)
        ]
    }
    return source, evidence, frozen


def test_build_uses_signed_cad_rays_and_removes_hypothesis_status():
    source, evidence, frozen = fixtures()
    result = MODULE.build_config(
        source,
        evidence,
        frozen,
        evidence_sha256="a" * 64,
        frozen_sha256="b" * 64,
        validation_sha256="c" * 64,
    )
    by_motor = {row["motor"]: row for row in result["rotors"]}
    frozen_by_motor = {row["motor"]: row for row in frozen["motors"]}
    for motor, record in by_motor.items():
        sign = evidence["motors"][motor - 1]["thrust_sign_along_cad_propeller_ray"]
        np.testing.assert_allclose(
            record["axis_body"],
            sign * np.asarray(frozen_by_motor[motor]["cad_propeller_side_axis"]),
            atol=1e-6,
        )
        assert record["thrust_sign_status"] == "RESOLVED_BY_VALIDATED_EVIDENCE"
    assert result["positive_thrust_direction_status"] == "RESOLVED_FOR_ALL_MOTORS"
    assert result["axis_assumption"] == "PHYSICAL_SIGNED_THRUST_AXES_FROZEN"
    assert "debug_hypothesis_vertical_capacity" not in result


def test_validation_hash_mismatch_is_rejected():
    with pytest.raises(ValueError, match="evidence JSON SHA-256"):
        MODULE.verify_validation(
            b"evidence",
            b"frozen",
            {
                "status": "MOTOR_THRUST_EVIDENCE_PASS",
                "promotion_allowed": True,
                "evidence_sha256": "bad",
                "frozen_status_sha256": MODULE.sha256(b"frozen"),
            },
        )
