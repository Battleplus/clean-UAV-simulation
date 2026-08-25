#!/usr/bin/env python3
"""Generate the 4 kg SO101 static flight-safety workspace envelope."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from drone_arm_sim.workspace_envelope import DEFAULT_GRID_COUNTS, scan_workspace


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    default_output = workspace / "analysis/base1/arm_workspace_envelope_4kg.json"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=package / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf",
    )
    parser.add_argument(
        "--motion-reference",
        type=Path,
        default=package / "config/so101_motion_reference_4kg.json",
    )
    parser.add_argument(
        "--flight-config",
        type=Path,
        default=package / "config/my_drone_v3_cad_debug_4kg.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output,
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Use a 3x3x3x3x3x2 smoke-test grid instead of the standard grid.",
    )
    parser.add_argument(
        "--include-samples",
        action="store_true",
        help="Include all per-pose records; the default report stores compact boundaries only.",
    )
    parser.add_argument(
        "--rotor-clearance-m",
        type=float,
        default=0.005,
        help="Clearance added around each CAD-derived rotor swept volume.",
    )
    parser.add_argument(
        "--joint-effort-reserve-fraction",
        type=float,
        default=0.10,
        help="Fraction of each documented servo effort kept in reserve.",
    )
    parser.add_argument(
        "--target-mass-kg",
        type=float,
        default=4.0,
        help="Aggregate mass used by the coupled model (default keeps Base 1 at 4 kg).",
    )
    parser.add_argument(
        "--gravity-torque-limit-nm",
        type=float,
        default=1.35,
        help="Static arm gravity-compensation acceptance limit.",
    )
    parser.add_argument(
        "--maximum-motor-delta-n",
        type=float,
        default=1.60,
        help="Maximum per-motor compensation overlay relative to folded hover.",
    )
    parser.add_argument(
        "--minimum-delta-headroom-n",
        type=float,
        default=None,
        help=(
            "Per-motor compensation authority kept in reserve.  When omitted, "
            "read candidate_compensation_limits.minimum_delta_headroom_n from "
            "the selected flight config (legacy/Base1 fallback: 0.05 N)."
        ),
    )
    args = parser.parse_args()
    if args.quick and args.output == default_output:
        # A smoke grid is deliberately too sparse to prove category coverage;
        # never let it overwrite the standard machine-readable envelope.
        args.output = default_output.with_name("arm_workspace_envelope_4kg_quick.json")
    counts = (
        {name: (2 if name == "gripper" else 3) for name in DEFAULT_GRID_COUNTS}
        if args.quick
        else DEFAULT_GRID_COUNTS
    )
    selected_config = json.loads(args.flight_config.read_text(encoding="utf-8"))
    configured_limits = selected_config.get("candidate_compensation_limits", {})
    if not isinstance(configured_limits, dict):
        configured_limits = {}
    minimum_delta_headroom_n = (
        float(configured_limits.get("minimum_delta_headroom_n", 0.05))
        if args.minimum_delta_headroom_n is None
        else float(args.minimum_delta_headroom_n)
    )
    report = scan_workspace(
        args.urdf,
        args.motion_reference,
        args.flight_config,
        target_mass_kg=args.target_mass_kg,
        grid_counts=counts,
        gravity_torque_limit_nm=args.gravity_torque_limit_nm,
        maximum_motor_delta_n=args.maximum_motor_delta_n,
        minimum_delta_headroom_n=minimum_delta_headroom_n,
        rotor_clearance_m=args.rotor_clearance_m,
        joint_effort_reserve_fraction=args.joint_effort_reserve_fraction,
        include_samples=args.include_samples,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    summary = report["summary"]
    if args.quick:
        passed = all(
            report["anchors"].get(name, {}).get("flight_allowed", False)
            for name in ("retracted", "flight_straight_forward")
        )
        marker = "ARM_WORKSPACE_SMOKE_PASS " if passed else "ARM_WORKSPACE_SMOKE_FAIL "
    else:
        passed = bool(summary["required_direction_coverage_complete"])
        marker = "ARM_WORKSPACE_SCAN_PASS " if passed else "ARM_WORKSPACE_SCAN_INCOMPLETE "
    print(
        marker + json.dumps(
            {
                "schema": report["schema"],
                "scan_mode": "quick_smoke" if args.quick else "standard_envelope",
                "sample_count": summary["sample_count"],
                "flight_allowed_count": summary["flight_allowed_count"],
                "coverage_complete": summary["required_direction_coverage_complete"],
                "coverage_checks": summary["coverage_checks"],
                "maximum_allowed_gravity_torque_nm": summary[
                    "maximum_allowed_gravity_torque_nm"
                ],
                "output": str(args.output),
            },
            sort_keys=True,
        )
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
