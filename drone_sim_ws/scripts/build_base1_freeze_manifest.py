#!/usr/bin/env python3
"""Freeze and verify the files that define the Base 1 flight behaviour."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


BASE1_REF = "base-1"
BASE1_COMMIT = "b340ed6"
CORE_PATHS = (
    "drone_sim_ws/px4/airframes/4027_gz_my_drone_octorotor_debug_4kg",
    "drone_sim_ws/scripts/run_ros2_dds_wasd.sh",
    "drone_sim_ws/scripts/start_main_model_gazebo.ps1",
    "drone_sim_ws/scripts/start_ros2_dds_wasd.ps1",
    "drone_sim_ws/scripts/wsl_start_ros2_dds_debug_4kg.sh",
    "drone_sim_ws/scripts/wsl_start_ros2_dds_noarm.sh",
    "drone_sim_ws/src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json",
    "drone_sim_ws/src/drone_arm_sim/drone_arm_sim/gazebo_direct_motor_model.py",
    "drone_sim_ws/src/drone_arm_sim/launch/cad_direct_thrust.launch.py",
    "drone_sim_ws/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf",
    "drone_sim_ws/src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf",
    "drone_sim_ws/src/drone_motor_system/src/LatestWrenchSystem.cc",
    "drone_sim_ws/src/px4_ros2_control/px4_ros2_control/dds_wasd_control.py",
)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_text(data: bytes) -> bytes:
    """Normalize line endings for the text-only frozen core file set."""
    return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def git_bytes(root: Path, ref: str, relative: str) -> bytes:
    return subprocess.check_output(
        ["git", "show", f"{ref}:{relative}"], cwd=root
    )


def git_commit(root: Path, ref: str) -> str:
    return subprocess.check_output(
        ["git", "rev-parse", f"{ref}^{{commit}}"],
        cwd=root,
        text=True,
        encoding="utf-8",
    ).strip()


def build_manifest(root: Path, ref: str) -> dict:
    commit = git_commit(root, ref)
    records = []
    mismatches = []
    for relative in CORE_PATHS:
        reference_hash = sha256(canonical_text(git_bytes(root, ref, relative)))
        local_path = root / relative
        local_payload = local_path.read_bytes() if local_path.is_file() else None
        local_hash = sha256(local_payload) if local_payload is not None else None
        local_canonical_hash = (
            sha256(canonical_text(local_payload)) if local_payload is not None else None
        )
        matches = local_canonical_hash == reference_hash
        records.append(
            {
                "path": relative,
                "base1_sha256": reference_hash,
                "worktree_raw_sha256": local_hash,
                "worktree_canonical_sha256": local_canonical_hash,
                "comparison": "SHA-256 after CRLF/CR to LF normalization",
                "matches_base1": matches,
            }
        )
        if not matches:
            mismatches.append(relative)

    return {
        "schema": "my_drone.base1-flight-freeze.v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "reference": ref,
        "reference_commit": commit,
        "expected_base1_commit_prefix": BASE1_COMMIT,
        "scope": "4kg Base 1 flight only; 7.735kg explicitly excluded",
        "default_compensation": {
            "ARM_FEEDFORWARD_ENABLED": False,
            "ARM_TORQUE_FEEDFORWARD_ENABLED": False,
            "ARM_STATIC_COM_FEEDFORWARD_GAIN": 0.0,
            "ARM_DISTURBANCE_OBSERVER_ENABLED": False,
        },
        "files": records,
        "all_core_files_match": not mismatches,
        "mismatches": mismatches,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", default=BASE1_REF)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("drone_sim_ws/baselines/Base_1_flight_freeze.json"),
    )
    parser.add_argument(
        "--allow-mismatch",
        action="store_true",
        help="write a diagnostic manifest without failing on changed core files",
    )
    args = parser.parse_args()

    root = Path(
        subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            text=True,
            encoding="utf-8",
        ).strip()
    )
    manifest = build_manifest(root, args.ref)
    output = args.output if args.output.is_absolute() else root / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"BASE1_FREEZE_MANIFEST {output}")
    print(f"BASE1_CORE_MATCH={str(manifest['all_core_files_match']).lower()}")
    if manifest["mismatches"]:
        print("BASE1_MISMATCHES=" + ",".join(manifest["mismatches"]))
    return 0 if manifest["all_core_files_match"] or args.allow_mismatch else 2


if __name__ == "__main__":
    raise SystemExit(main())
