#!/usr/bin/env python3
"""Report timing gaps in PX4 streams that can invalidate arm-flight tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


DEFAULT_TOPICS = (
    "sensor_combined",
    "vehicle_status",
    "vehicle_local_position",
    "vehicle_attitude",
    "actuator_motors",
    "vehicle_visual_odometry",
    "vehicle_local_position_groundtruth",
)


def analyze(path: Path, topics: tuple[str, ...] = DEFAULT_TOPICS) -> dict:
    ulog = ULog(str(path))
    report = {"ulog": str(path), "streams": {}, "sensor_timeout_messages": []}
    for name in topics:
        candidates = [item for item in ulog.data_list if item.name == name]
        if not candidates:
            report["streams"][name] = {"present": False}
            continue
        stamps = np.asarray(candidates[0].data["timestamp"], dtype=np.int64)
        gaps = np.diff(stamps) * 1.0e-6
        index = int(np.argmax(gaps)) if gaps.size else None
        report["streams"][name] = {
            "present": True,
            "samples": int(stamps.size),
            "max_gap_s": float(gaps[index]) if index is not None else 0.0,
            "gap_start_s": float(stamps[index] * 1.0e-6) if index is not None else None,
            "gap_end_s": float(stamps[index + 1] * 1.0e-6) if index is not None else None,
        }
    for message in ulog.logged_messages:
        text = str(message.message)
        if "TIMEOUT" in text.upper() or "SENSOR" in text.upper() and "FAIL" in text.upper():
            report["sensor_timeout_messages"].append(
                {"timestamp_s": float(message.timestamp * 1.0e-6), "message": text}
            )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = analyze(args.ulog)
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
