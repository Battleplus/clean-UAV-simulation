#!/usr/bin/env python3
"""Build a new V3 motor config only from validated signed-thrust evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def verify_validation(
    evidence_payload: bytes,
    frozen_payload: bytes,
    validation: dict,
) -> None:
    if validation.get("status") != "MOTOR_THRUST_EVIDENCE_PASS":
        raise ValueError("motor thrust evidence validation is not PASS")
    if validation.get("promotion_allowed") is not True:
        raise ValueError("motor thrust evidence does not allow promotion")
    if validation.get("evidence_sha256") != sha256(evidence_payload):
        raise ValueError("evidence JSON SHA-256 does not match validation")
    if validation.get("frozen_status_sha256") != sha256(frozen_payload):
        raise ValueError("frozen geometry SHA-256 does not match validation")


def build_config(
    source: dict,
    evidence: dict,
    frozen: dict,
    *,
    evidence_sha256: str,
    frozen_sha256: str,
    validation_sha256: str,
) -> dict:
    config = json.loads(json.dumps(source))
    evidence_by_motor = {int(row["motor"]): row for row in evidence["motors"]}
    frozen_by_motor = {int(row["motor"]): row for row in frozen["motors"]}
    if set(evidence_by_motor) != set(range(1, 9)):
        raise ValueError("evidence must contain motors 1..8")
    if set(frozen_by_motor) != set(range(1, 9)):
        raise ValueError("frozen geometry must contain motors 1..8")

    maximum_thrust = float(frozen["maximum_thrust_per_motor_n"])
    if not math.isclose(maximum_thrust, 11.76798, abs_tol=1.0e-6):
        raise ValueError("frozen maximum thrust is not 11.76798 N")
    for rotor in config["rotors"]:
        motor = int(rotor["motor"])
        record = evidence_by_motor[motor]
        frozen_motor = frozen_by_motor[motor]
        ray = np.asarray(frozen_motor["cad_propeller_side_axis"], dtype=float)
        if ray.shape != (3,) or not np.all(np.isfinite(ray)):
            raise ValueError(f"motor {motor}: invalid frozen CAD ray")
        norm = float(np.linalg.norm(ray))
        if not math.isclose(norm, 1.0, abs_tol=2.0e-6):
            raise ValueError(f"motor {motor}: CAD ray is not unit length")
        sign = int(record["thrust_sign_along_cad_propeller_ray"])
        rotor["axis_body"] = (sign * ray / norm).tolist()
        rotor["cad_propeller_side_axis"] = (ray / norm).tolist()
        rotor["axis_role"] = "SIGNED_THRUST_AXIS_FROM_VALIDATED_PHYSICAL_EVIDENCE"
        rotor["thrust_sign_status"] = "RESOLVED_BY_VALIDATED_EVIDENCE"
        rotor["thrust_sign_along_cad_propeller_ray"] = sign
        rotor["prop_type"] = record["prop_type"]
        rotor["propeller_part_number"] = record.get("propeller_part_number")
        rotor["evidence_type"] = record["evidence_type"]
        rotor["vertical_thrust_direction"] = (
            "upward" if rotor["axis_body"][2] < 0.0 else "downward"
        )
        rotor.pop("opposite_pitch_required_if_all_up", None)

    axes = np.asarray([rotor["axis_body"] for rotor in config["rotors"]], dtype=float)
    full_throttle_signed_upward = maximum_thrust * float(-np.sum(axes[:, 2]))
    controllable_upward = maximum_thrust * float(np.sum(np.maximum(-axes[:, 2], 0.0)))
    mass = float(config.get("temporary_fixed_mass_kg", config["estimated_all_up_mass_kg"]))
    config.update(
        {
            "description": "V3 signed motor thrust axes promoted from validated physical evidence; see physical_evidence_provenance",
            "scenario": "physical_signed_thrust_axes_evidence_frozen",
            "axis_assumption": "PHYSICAL_SIGNED_THRUST_AXES_FROZEN",
            "positive_thrust_direction_status": "RESOLVED_FOR_ALL_MOTORS",
            "maximum_thrust_n": maximum_thrust,
            "full_throttle_signed_vertical_force_n": full_throttle_signed_upward,
            "maximum_vertical_force_n": controllable_upward,
            "maximum_supported_mass_kg": controllable_upward / 9.80665,
            "estimated_vertical_thrust_to_weight": controllable_upward / (mass * 9.80665),
            "flight_feasibility_nonreversible": (
                "FEASIBLE_BY_STATIC_VERTICAL_THRUST"
                if controllable_upward > mass * 9.80665
                else "INFEASIBLE_BY_STATIC_VERTICAL_THRUST"
            ),
            "physical_evidence_provenance": {
                "evidence_sha256": evidence_sha256,
                "frozen_geometry_sha256": frozen_sha256,
                "validation_sha256": validation_sha256,
                "promotion_policy": "generated as a new file; source configuration was not overwritten",
            },
        }
    )
    config.pop("debug_hypothesis_vertical_capacity", None)
    return config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--frozen-status", required=True, type=Path)
    parser.add_argument("--validation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    paths = [path.resolve() for path in (args.source, args.evidence, args.frozen_status, args.validation)]
    output = args.output.resolve()
    if output in paths or output.exists():
        raise SystemExit("refusing to overwrite an input or existing output")
    source_payload, evidence_payload, frozen_payload, validation_payload = [
        path.read_bytes() for path in paths
    ]
    validation = json.loads(validation_payload.decode("utf-8"))
    verify_validation(evidence_payload, frozen_payload, validation)
    config = build_config(
        json.loads(source_payload.decode("utf-8")),
        json.loads(evidence_payload.decode("utf-8")),
        json.loads(frozen_payload.decode("utf-8")),
        evidence_sha256=sha256(evidence_payload),
        frozen_sha256=sha256(frozen_payload),
        validation_sha256=sha256(validation_payload),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
