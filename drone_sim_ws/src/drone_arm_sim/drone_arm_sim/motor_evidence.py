"""Validation rules for freezing signed motor thrust directions.

CAD cylinder axes are signless geometric facts.  This module deliberately
requires one auditable handedness or signed-force record per motor before a
caller may promote those axes to signed thrust directions.
"""

from __future__ import annotations

import math
from typing import Any


EVIDENCE_TYPES = {"manufacturer", "bench_test", "pitch_accurate_cad"}
PROP_TYPES = {"NORMAL", "REVERSE"}
ROTATION_OBSERVATIONS = {
    "LOOKING_FROM_MOTOR_TOWARD_PROPELLER",
    "LOOKING_ALONG_CAD_PROPELLER_SIDE_RAY_TOWARD_MOTOR",
}
FORCE_POSITIVE_AXES = {
    "CAD_PROPELLER_SIDE_RAY": 1,
    "OPPOSITE_CAD_PROPELLER_SIDE_RAY": -1,
}


def validate_motor_thrust_evidence(
    evidence: dict[str, Any], frozen_status: dict[str, Any]
) -> list[str]:
    """Return deterministic validation errors; an empty list closes the gate."""

    errors: list[str] = []
    records = evidence.get("motors")
    if not isinstance(records, list):
        return ["motors must be a list"]
    by_motor: dict[int, dict[str, Any]] = {}
    for record in records:
        if not isinstance(record, dict) or not isinstance(record.get("motor"), int):
            errors.append("every motor record must contain an integer motor")
            continue
        motor = int(record["motor"])
        if motor in by_motor:
            errors.append(f"motor {motor}: duplicate record")
        by_motor[motor] = record

    frozen_by_motor = {
        int(record["motor"]): record for record in frozen_status.get("motors", [])
    }
    if set(frozen_by_motor) != set(range(1, 9)):
        errors.append("frozen status must contain motors 1..8")
        return errors
    missing = sorted(set(range(1, 9)) - set(by_motor))
    extra = sorted(set(by_motor) - set(range(1, 9)))
    if missing:
        errors.append(f"missing motor records: {missing}")
    if extra:
        errors.append(f"unexpected motor records: {extra}")

    for motor in range(1, 9):
        if motor not in by_motor:
            continue
        record = by_motor[motor]
        frozen = frozen_by_motor[motor]
        prefix = f"motor {motor}:"
        if record.get("rotation") != frozen.get("rotation"):
            errors.append(
                f"{prefix} rotation must match frozen {frozen.get('rotation')}"
            )
        if record.get("prop_type") not in PROP_TYPES:
            errors.append(f"{prefix} prop_type must be NORMAL or REVERSE")
        if record.get("thrust_sign_along_cad_propeller_ray") not in (-1, 1):
            errors.append(
                f"{prefix} thrust_sign_along_cad_propeller_ray must be -1 or 1"
            )
        evidence_type = record.get("evidence_type")
        if evidence_type not in EVIDENCE_TYPES:
            errors.append(
                f"{prefix} evidence_type must be one of {sorted(EVIDENCE_TYPES)}"
            )
        refs = record.get("evidence_refs")
        if (
            not isinstance(refs, list)
            or not refs
            or any(not isinstance(value, str) or not value.strip() for value in refs)
        ):
            errors.append(f"{prefix} evidence_refs must contain at least one reference")
        if record.get("rotation_observation") not in ROTATION_OBSERVATIONS:
            errors.append(
                f"{prefix} rotation_observation must freeze the CW/CCW viewing direction"
            )
        max_thrust = record.get("maximum_thrust_n")
        if not isinstance(max_thrust, (int, float)) or not math.isclose(
            float(max_thrust),
            float(frozen_status.get("maximum_thrust_per_motor_n", math.nan)),
            abs_tol=1e-6,
        ):
            errors.append(f"{prefix} maximum_thrust_n must be the frozen 11.76798 N")

        if evidence_type == "bench_test":
            if not isinstance(record.get("rpm"), (int, float)) or float(record["rpm"]) <= 0:
                errors.append(f"{prefix} bench_test requires rpm > 0")
            if (
                not isinstance(record.get("signed_axial_force_n"), (int, float))
                or abs(float(record["signed_axial_force_n"])) <= 0
            ):
                errors.append(f"{prefix} bench_test requires nonzero signed_axial_force_n")
            if not str(record.get("observation_definition", "")).strip():
                errors.append(f"{prefix} bench_test requires observation_definition")
            force_axis = record.get("force_positive_axis")
            if force_axis not in FORCE_POSITIVE_AXES:
                errors.append(
                    f"{prefix} bench_test force_positive_axis must be one of "
                    f"{sorted(FORCE_POSITIVE_AXES)}"
                )
            elif isinstance(record.get("signed_axial_force_n"), (int, float)):
                measured_sign = (
                    1 if float(record["signed_axial_force_n"]) > 0 else -1
                ) * FORCE_POSITIVE_AXES[force_axis]
                if record.get("thrust_sign_along_cad_propeller_ray") != measured_sign:
                    errors.append(
                        f"{prefix} thrust sign contradicts signed axial force and force axis"
                    )
        elif evidence_type == "manufacturer":
            if not str(record.get("propeller_part_number", "")).strip():
                errors.append(f"{prefix} manufacturer evidence requires propeller_part_number")
            if not str(record.get("observation_definition", "")).strip():
                errors.append(
                    f"{prefix} manufacturer evidence requires a thrust/handedness definition"
                )
        elif evidence_type == "pitch_accurate_cad":
            signed_pitch = record.get("signed_pitch_deg")
            if (
                not isinstance(signed_pitch, (int, float))
                or abs(float(signed_pitch)) < 0.1
            ):
                errors.append(
                    f"{prefix} pitch_accurate_cad requires |signed_pitch_deg| >= 0.1"
                )
            if not str(record.get("pitch_sign_definition", "")).strip():
                errors.append(
                    f"{prefix} pitch_accurate_cad requires pitch_sign_definition"
                )

    return errors
