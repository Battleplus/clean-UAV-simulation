#!/usr/bin/env python3
"""Deterministic compute-rate and legacy-equivalence audit for Base 1 estimator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--motion-reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--calls-per-trial", type=int, default=100)
    args = parser.parse_args()
    dynamics = CoupledArmDynamics(
        args.urdf, args.motion_reference, target_mass_kg=4.0
    )
    positions = dict(zip(JOINT_NAMES, [0.2, -0.4, 0.6, -0.3, 0.2, 0.4]))
    velocities = dict(zip(JOINT_NAMES, [0.1, -0.08, 0.06, -0.04, 0.02, 0.01]))
    accelerations = dict(zip(JOINT_NAMES, [0.2, -0.15, 0.1, -0.08, 0.05, 0.02]))

    optimized = dynamics.state(positions, velocities, accelerations)
    reference = dynamics._state_finite_difference_reference(
        positions, velocities, accelerations
    )
    errors = {
        "com_m": float(np.max(np.abs(
            optimized.center_of_mass_m - reference.center_of_mass_m
        ))),
        "inertia_kg_m2": float(np.max(np.abs(
            optimized.inertia_at_com_kg_m2 - reference.inertia_at_com_kg_m2
        ))),
        "reaction_force_n": float(np.max(np.abs(
            optimized.reaction_force_body_n - reference.reaction_force_body_n
        ))),
        "reaction_torque_nm": float(np.max(np.abs(
            optimized.reaction_torque_body_nm - reference.reaction_torque_body_nm
        ))),
    }
    elapsed_ms = []
    for _ in range(args.trials):
        for _ in range(10):
            dynamics.state(positions, velocities, accelerations)
        start = time.perf_counter()
        for _ in range(args.calls_per_trial):
            dynamics.state(positions, velocities, accelerations)
        elapsed_ms.append(
            1000.0 * (time.perf_counter() - start) / args.calls_per_trial
        )
    worst_hz = 1000.0 / max(elapsed_ms)
    gates = {
        "worst_trial_at_least_125_hz": worst_hz >= 125.0,
        "com_equivalent": errors["com_m"] <= 1.0e-12,
        "inertia_equivalent": errors["inertia_kg_m2"] <= 1.0e-12,
        "force_equivalent": errors["reaction_force_n"] <= 1.0e-7,
        "torque_equivalent": errors["reaction_torque_nm"] <= 1.0e-7,
    }
    report = {
        "schema": "my_drone.base1-coupling-estimator-benchmark.v1",
        "scope": "Base 1 4kg only; read-only estimator; 7.735kg not exercised",
        "requested_runtime_rate_hz": 100.0,
        "trial_ms_per_call": elapsed_ms,
        "worst_trial_hz": worst_hz,
        "legacy_equivalence_max_abs_error": errors,
        "gates": gates,
        "pass": all(gates.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
