#!/usr/bin/env python3
"""Write an immutable configuration manifest for one arm-DOB flight case."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


def file_record(path: Path) -> dict:
    resolved = path.resolve()
    payload = resolved.read_bytes()
    return {
        "path": str(resolved),
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def build_manifest(
    *,
    label: str,
    observer_enabled: bool,
    observer_gain: float,
    files: dict[str, Path],
) -> dict:
    gain = float(observer_gain)
    if not math.isfinite(gain) or not 0.0 < gain <= 0.5:
        raise ValueError("observer gain must be finite and in (0, 0.5]")
    required = {
        "config",
        "urdf",
        "world",
        "airframe",
        "backend_launcher",
        "flight_driver",
        "observer_source",
        "motor_model_source",
        "coupling_monitor_source",
        "px4_binary",
    }
    if set(files) != required:
        raise ValueError(f"manifest files must be exactly {sorted(required)}")
    return {
        "schema": 1,
        "label": str(label),
        "observer_enabled": bool(observer_enabled),
        "common": {
            "profile": "full_extend_slow_4kg",
            "headless": True,
            "fresh_px4_workdir": True,
            "gz_random_seed": 4027,
            "model_settle_s": 8.0,
            "model_settle_hold_s": 2.0,
            "px4_ready_settle_s": 5.0,
            "px4_ready_stable_hold_s": 5.0,
            "arm_ground_stable_hold_s": 5.0,
            "predictive_torque_feedforward_enabled": False,
            "static_com_feedforward_gain": 0.0,
            "observer_gain": gain,
            "observer_max_torque_nm": 0.08,
            "observer_max_delta_n": 1.0,
            "files": {
                name: file_record(path) for name, path in sorted(files.items())
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--label", required=True)
    parser.add_argument("--observer-enabled", choices=("true", "false"), required=True)
    parser.add_argument("--observer-gain", type=float, required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--airframe", required=True, type=Path)
    parser.add_argument("--backend-launcher", required=True, type=Path)
    parser.add_argument("--flight-driver", required=True, type=Path)
    parser.add_argument("--observer-source", required=True, type=Path)
    parser.add_argument("--motor-model-source", required=True, type=Path)
    parser.add_argument("--coupling-monitor-source", required=True, type=Path)
    parser.add_argument("--px4-binary", required=True, type=Path)
    args = parser.parse_args()
    manifest = build_manifest(
        label=args.label,
        observer_enabled=args.observer_enabled == "true",
        observer_gain=args.observer_gain,
        files={
            "config": args.config,
            "urdf": args.urdf,
            "world": args.world,
            "airframe": args.airframe,
            "backend_launcher": args.backend_launcher,
            "flight_driver": args.flight_driver,
            "observer_source": args.observer_source,
            "motor_model_source": args.motor_model_source,
            "coupling_monitor_source": args.coupling_monitor_source,
            "px4_binary": args.px4_binary,
        },
    )
    rendered = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
