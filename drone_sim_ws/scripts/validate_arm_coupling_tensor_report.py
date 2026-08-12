#!/usr/bin/env python3
"""Validate a mass-specific arm coupling tensor report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


REQUIRED_STATES = (
    "retracted",
    "expanded_static_work_a",
    "moving_work_a",
    "moving_work_a_with_payload",
)


def validate(path: Path, expected_mass_kg: float, expected_payload_kg: float) -> dict:
    payload = path.read_bytes()
    report = json.loads(payload.decode("utf-8"))
    base_mass = float(report.get("model_mass_without_payload_kg", math.nan))
    payload_mass = float(report.get("payload_mass_kg", math.nan))
    if not math.isclose(base_mass, expected_mass_kg, abs_tol=1.0e-9):
        raise ValueError(f"base mass {base_mass} does not equal {expected_mass_kg}")
    if not math.isclose(payload_mass, expected_payload_kg, abs_tol=1.0e-9):
        raise ValueError(
            f"payload mass {payload_mass} does not equal {expected_payload_kg}"
        )
    states = report.get("states", {})
    if tuple(states) != REQUIRED_STATES:
        raise ValueError("required coupling states are missing or out of order")

    tensors: dict[str, np.ndarray] = {}
    summaries: dict[str, dict] = {}
    for name in REQUIRED_STATES:
        state = states[name]
        tensor = np.asarray(state.get("inertia_at_com_kg_m2"), dtype=float)
        if tensor.shape != (3, 3) or not np.all(np.isfinite(tensor)):
            raise ValueError(f"{name}: inertia tensor must be finite 3x3")
        if not np.allclose(tensor, tensor.T, atol=1.0e-9):
            raise ValueError(f"{name}: inertia tensor is not symmetric")
        eigenvalues = np.linalg.eigvalsh(tensor)
        if np.min(eigenvalues) <= 0.0:
            raise ValueError(f"{name}: inertia tensor is not positive definite")
        tensors[name] = tensor
        summaries[name] = {
            "mass_kg": float(state["mass_kg"]),
            "com_shift_norm_m": float(state["com_shift_norm_m"]),
            "inertia_eigenvalues_kg_m2": eigenvalues.tolist(),
            "off_diagonal_norm_kg_m2": float(
                np.linalg.norm(tensor[np.triu_indices(3, 1)])
            ),
            "reaction_torque_norm_nm": float(state["reaction_torque_norm_nm"]),
        }

    for name in REQUIRED_STATES[:3]:
        if not math.isclose(summaries[name]["mass_kg"], base_mass, abs_tol=1.0e-9):
            raise ValueError(f"{name}: mass is inconsistent with the base model")
    expected_loaded = base_mass + payload_mass
    if not math.isclose(
        summaries["moving_work_a_with_payload"]["mass_kg"],
        expected_loaded,
        abs_tol=1.0e-9,
    ):
        raise ValueError("loaded state mass does not equal base plus payload")
    if np.linalg.norm(tensors["expanded_static_work_a"] - tensors["retracted"]) <= 1.0e-4:
        raise ValueError("arm pose does not produce a measurable inertia change")
    if not np.allclose(
        tensors["expanded_static_work_a"], tensors["moving_work_a"], atol=1.0e-12
    ):
        raise ValueError("equal-pose static and moving states disagree on inertia")
    if np.linalg.norm(
        tensors["moving_work_a_with_payload"] - tensors["moving_work_a"]
    ) <= 1.0e-4:
        raise ValueError("payload does not produce a measurable inertia change")
    if summaries["moving_work_a"]["reaction_torque_norm_nm"] <= 0.01:
        raise ValueError("moving state does not contain a meaningful reaction torque")

    return {
        "result": "ARM_COUPLING_TENSOR_REPORT_PASS",
        "source": str(path.resolve()),
        "source_sha256": hashlib.sha256(payload).hexdigest(),
        "base_mass_kg": base_mass,
        "payload_mass_kg": payload_mass,
        "states": summaries,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("--expected-mass-kg", type=float, required=True)
    parser.add_argument("--expected-payload-kg", type=float, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(
        args.report.resolve(), args.expected_mass_kg, args.expected_payload_kg
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
