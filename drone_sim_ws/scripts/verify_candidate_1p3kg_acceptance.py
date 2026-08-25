#!/usr/bin/env python3
"""Audit offline evidence and strict dynamic markers for the 1.3 kg candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ARM_LINKS = {
    "arm_base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "moving_jaw_link",
}
DIRECTIONS = (
    "front", "rear", "left", "right", "up", "down",
    "front_left", "front_right", "rear_left", "rear_right",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _exit_zero(path: Path) -> bool:
    return path.is_file() and path.read_text(encoding="utf-8").strip() == "0"


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report",
        type=Path,
        default=workspace / "analysis/base1/candidate_1p3kg_acceptance_status.json",
    )
    args = parser.parse_args()
    paths = {
        "urdf": package / "urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf",
        "motion_reference": package / "config/so101_motion_reference_4kg.json",
        "flight_config": package / "config/my_drone_v3_cad_candidate_1p3kg.json",
        "envelope": workspace / "analysis/base1/arm_workspace_envelope_1p3kg.json",
        "plan": workspace / "analysis/base1/directional_workspace_flight_plan_1p3kg.json",
        "planner": workspace / "scripts/plan_directional_workspace_acceptance_4kg.py",
        "trajectory_preflight": package / "drone_arm_sim/trajectory_preflight.py",
        "workspace_envelope": package / "drone_arm_sim/workspace_envelope.py",
    }
    missing = sorted(name for name, path in paths.items() if not path.is_file())
    checks: dict[str, bool] = {"all_required_files_present": not missing}
    details: dict = {"missing_files": missing}
    if missing:
        report = {
            "status": "INCOMPLETE",
            "offline_ready": False,
            "dynamic_accepted": False,
            "checks": checks,
            "details": details,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("CANDIDATE_1P3KG_AUDIT_INCOMPLETE " + json.dumps(report, sort_keys=True))
        return 2

    root = ET.parse(paths["urdf"]).getroot()
    masses = {
        link.get("name"): float(link.find("inertial/mass").get("value"))
        for link in root.findall("link")
        if link.find("inertial/mass") is not None
    }
    arm_mass = sum(value for name, value in masses.items() if name in ARM_LINKS)
    airframe_mass = sum(value for name, value in masses.items() if name not in ARM_LINKS)
    total_mass = arm_mass + airframe_mass
    config = json.loads(paths["flight_config"].read_text(encoding="utf-8"))
    envelope = json.loads(paths["envelope"].read_text(encoding="utf-8"))
    plan = json.loads(paths["plan"].read_text(encoding="utf-8"))
    checks.update(
        {
            "total_mass_1p3kg": abs(total_mass - 1.3) <= 1.0e-9,
            "arm_mass_0p6kg": abs(arm_mass - 0.6) <= 1.0e-9,
            "airframe_mass_0p7kg": abs(airframe_mass - 0.7) <= 1.0e-9,
            "config_mass_matches": abs(float(config["estimated_all_up_mass_kg"]) - total_mass) <= 1.0e-9,
            "envelope_status_static": envelope.get("status") == "DISCRETE_STATIC_ENVELOPE",
            "envelope_full_direction_coverage": bool(
                envelope.get("summary", {}).get("required_direction_coverage_complete")
            ),
            "envelope_strict_gravity_limit": abs(float(envelope["limits"]["gravity_torque_limit_nm"]) - 0.90) <= 1.0e-12,
            "envelope_strict_motor_delta": abs(float(envelope["limits"]["maximum_motor_delta_n"]) - 1.25) <= 1.0e-12,
            "envelope_has_delta_headroom": float(envelope["summary"]["minimum_allowed_overlay_delta_headroom_n"]) >= 0.05,
            "plan_is_preflight_only": plan.get("status") == "PREFLIGHTED_NOT_FLIGHT_ACCEPTED",
            "plan_has_ten_directions": tuple(plan.get("direction_order", ())) == DIRECTIONS,
            "plan_all_legs_return_home": all(
                leg.get("outward", {}).get("decision")
                in {"accepted", "slowed", "shortened"}
                and leg.get("return", {}).get("decision")
                in {"accepted", "slowed"}
                and float(leg.get("return", {}).get("distance_scale", 0.0)) == 1.0
                for leg in plan.get("legs", ())
            ),
        }
    )
    input_hashes = envelope.get("inputs", {}).get("sha256", {})
    checks["envelope_input_hashes_current"] = all(
        input_hashes.get(name) == sha256(paths[name])
        for name in ("urdf", "motion_reference", "flight_config")
    )
    evidence = plan.get("source_evidence", {})
    checks["plan_envelope_hash_current"] = (
        evidence.get("envelope_sha256") == sha256(paths["envelope"])
    )
    checks["plan_model_hashes_current"] = all(
        evidence.get("model_sha256", {}).get(name) == sha256(paths[name])
        for name in ("urdf", "motion_reference", "flight_config")
    )
    checks["plan_implementation_hashes_current"] = all(
        evidence.get("implementation_sha256", {}).get(name) == sha256(paths[name])
        for name in ("planner", "trajectory_preflight", "workspace_envelope")
    )
    offline_ready = all(checks.values())

    wasd_log = workspace / "analysis/base1/wasd_flight_acceptance_1p3kg.log"
    wasd_exit = workspace / "analysis/base1/wasd_flight_acceptance_1p3kg.exit"
    directional_log = workspace / "analysis/base1/directional_workspace_flight_acceptance_1p3kg.log"
    directional_exit = workspace / "analysis/base1/directional_workspace_flight_acceptance_1p3kg.exit"
    wasd_text = wasd_log.read_text(encoding="utf-8", errors="replace") if wasd_log.is_file() else ""
    directional_text = (
        directional_log.read_text(encoding="utf-8", errors="replace")
        if directional_log.is_file() else ""
    )
    stage_passes = re.findall(r"DIRECTIONAL_FLIGHT_STAGE_METRICS .* pass=True", directional_text)
    direction_passes = re.findall(r"DIRECTIONAL_FLIGHT_DIRECTION_RESULT .* pass=True", directional_text)
    dynamic_checks = {
        "wasd_exit_zero": _exit_zero(wasd_exit),
        "wasd_pass_marker": "DDS_WASD_PTY_PASS" in wasd_text,
        "directional_exit_zero": _exit_zero(directional_exit),
        "directional_pass_marker": "DDS_ARM_FLIGHT_PASS" in directional_text,
        "all_30_directional_stages_pass": len(stage_passes) == 30,
        "all_10_directions_return_and_pass": len(direction_passes) == 10,
    }
    dynamic_accepted = offline_ready and all(dynamic_checks.values())
    details.update(
        {
            "mass_kg": {
                "total": total_mass,
                "arm": arm_mass,
                "remaining_aircraft": airframe_mass,
            },
            "static_envelope": envelope.get("summary", {}),
            "dynamic_checks": dynamic_checks,
        }
    )
    report = {
        "status": (
            "DYNAMIC_ACCEPTED" if dynamic_accepted
            else "OFFLINE_READY_DYNAMIC_NOT_ACCEPTED" if offline_ready
            else "OFFLINE_EVIDENCE_STALE_OR_FAILED"
        ),
        "offline_ready": offline_ready,
        "dynamic_accepted": dynamic_accepted,
        "checks": checks,
        "details": details,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    marker = (
        "CANDIDATE_1P3KG_DYNAMIC_ACCEPTED"
        if dynamic_accepted
        else "CANDIDATE_1P3KG_OFFLINE_READY_DYNAMIC_NOT_ACCEPTED"
        if offline_ready
        else "CANDIDATE_1P3KG_AUDIT_FAILED"
    )
    print(marker + " " + json.dumps(report, sort_keys=True))
    return 0 if offline_ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
