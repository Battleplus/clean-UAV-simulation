#!/usr/bin/env python3
"""Validate physical propeller/thrust-sign evidence without mutating the model."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from drone_arm_sim.motor_evidence import validate_motor_thrust_evidence


def validate_reference_files(
    evidence: dict, evidence_path: Path
) -> tuple[list[str], list[dict[str, object]]]:
    """Require immutable local snapshots for every declared evidence reference."""

    errors: list[str] = []
    manifest: list[dict[str, object]] = []
    seen: set[Path] = set()
    for record in evidence.get("motors", []):
        if not isinstance(record, dict):
            continue
        motor = record.get("motor", "?")
        for raw_ref in record.get("evidence_refs", []) or []:
            if not isinstance(raw_ref, str) or not raw_ref.strip():
                continue
            if "://" in raw_ref:
                errors.append(
                    f"motor {motor}: evidence reference must be a local snapshot, not URL {raw_ref!r}"
                )
                continue
            candidate = Path(raw_ref)
            path = candidate if candidate.is_absolute() else evidence_path.parent / candidate
            path = path.resolve()
            if not path.is_file():
                errors.append(f"motor {motor}: evidence file does not exist: {raw_ref}")
                continue
            if path in seen:
                continue
            seen.add(path)
            payload = path.read_bytes()
            try:
                relative = path.relative_to(evidence_path.parent).as_posix()
            except ValueError:
                relative = str(path)
            manifest.append(
                {
                    "path": relative,
                    "size_bytes": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            )
    return errors, sorted(manifest, key=lambda row: str(row["path"]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence", type=Path)
    parser.add_argument(
        "--frozen-status",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "analysis"
            / "cad_direct"
            / "motor_physical_freeze_status.json"
        ),
    )
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    evidence_path = args.evidence.resolve()
    evidence_payload = evidence_path.read_bytes()
    frozen_path = args.frozen_status.resolve()
    frozen_payload = frozen_path.read_bytes()
    evidence = json.loads(evidence_payload.decode("utf-8"))
    frozen = json.loads(frozen_payload.decode("utf-8"))
    errors = validate_motor_thrust_evidence(evidence, frozen)
    reference_errors, reference_manifest = validate_reference_files(
        evidence, evidence_path
    )
    errors.extend(reference_errors)
    report = {
        "status": "MOTOR_THRUST_EVIDENCE_PASS" if not errors else "INCOMPLETE",
        "evidence": str(evidence_path),
        "evidence_size_bytes": len(evidence_payload),
        "evidence_sha256": hashlib.sha256(evidence_payload).hexdigest(),
        "frozen_status": str(frozen_path),
        "frozen_status_size_bytes": len(frozen_payload),
        "frozen_status_sha256": hashlib.sha256(frozen_payload).hexdigest(),
        "reference_manifest": reference_manifest,
        "errors": errors,
        "promotion_allowed": not errors,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
