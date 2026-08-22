#!/usr/bin/env python3
"""Measure the live Base 1 read-only coupling estimator and validate its schema."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUIRED_FIELDS = {
    "schema",
    "estimator_mode",
    "estimator_valid",
    "source_fresh",
    "source_age_s",
    "source_timeout_s",
    "source_joint_state_stamp_s",
    "output_stamp_s",
    "output_frame",
    "mass_kg",
    "com_body_flu_m",
    "inertia_tensor_kg_m2",
    "reaction_force_body_n",
    "reaction_torque_body_nm",
    "gravity_shift_torque_body_nm",
}


class Audit(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("base1_coupling_estimator_runtime_audit")
        self.receipts: list[float] = []
        self.reports: list[dict] = []
        self.create_subscription(
            String, topic, self.on_state, 1000
        )

    def on_state(self, message: String) -> None:
        try:
            report = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self.receipts.append(time.monotonic())
        self.reports.append(report)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument(
        "--topic", default="/my_drone/base1_estimator/coupling_state"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Audit(args.topic)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    intervals = np.diff(node.receipts)
    healthy = [report for report in node.reports if report.get("estimator_valid")]
    invalid = [report for report in node.reports if not report.get("estimator_valid")]
    complete = [REQUIRED_FIELDS <= set(report) for report in healthy]
    ages = [float(report["source_age_s"]) for report in healthy]
    rate_hz = (
        (len(node.receipts) - 1) / (node.receipts[-1] - node.receipts[0])
        if len(node.receipts) >= 2 else 0.0
    )
    latest = healthy[-1] if healthy else {}
    finite_arrays = all(
        np.all(np.isfinite(np.asarray(latest.get(name, []), dtype=float)))
        for name in (
            "com_body_flu_m",
            "inertia_tensor_kg_m2",
            "reaction_force_body_n",
            "reaction_torque_body_nm",
            "gravity_shift_torque_body_nm",
        )
    )
    gates = {
        "runtime_rate_at_least_95_hz": rate_hz >= 95.0,
        "at_least_500_samples": len(node.receipts) >= 500,
        "healthy_fraction_at_least_99_percent": (
            bool(node.reports) and len(healthy) / len(node.reports) >= 0.99
        ),
        "invalid_samples_fail_closed": all(
            report.get("source_fresh") is False
            and "reaction_force_body_n" not in report
            and "reaction_torque_body_nm" not in report
            for report in invalid
        ),
        "schema_complete": bool(complete) and all(complete),
        "read_only_mode": bool(healthy) and all(
            report.get("estimator_mode") == "read_only" for report in healthy
        ),
        "base_link_flu_frame": bool(healthy) and all(
            report.get("output_frame") == "base_link_flu" for report in healthy
        ),
        "source_age_within_100ms": bool(ages) and max(ages) <= 0.10,
        "latest_dynamic_arrays_finite": finite_arrays,
        "mass_is_4kg": abs(float(latest.get("mass_kg", float("nan"))) - 4.0) <= 1.0e-8,
    }
    report = {
        "schema": "my_drone.base1-coupling-estimator-runtime-audit.v1",
        "scope": "Base 1 4kg only; all compensation disabled; 7.735kg not exercised",
        "duration_s": args.duration,
        "samples": len(node.receipts),
        "valid_samples": len(healthy),
        "invalid_samples": len(invalid),
        "valid_fraction": len(healthy) / len(node.reports) if node.reports else 0.0,
        "measured_rate_hz": rate_hz,
        "interval_p50_ms": float(np.percentile(intervals, 50)) * 1000 if len(intervals) else None,
        "interval_p99_ms": float(np.percentile(intervals, 99)) * 1000 if len(intervals) else None,
        "max_source_age_ms": max(ages) * 1000 if ages else None,
        "latest_state": latest,
        "gates": gates,
        "pass": all(gates.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
