#!/usr/bin/env python3
"""Build a disposable 4 kg PX4 airframe from one bounded tuning decision."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


PARAM_RE = re.compile(
    r"^(\s*param\s+(?:set-default|set)\s+)([A-Z0-9_]+)(\s+)([-+0-9.eE]+)(\s*)$"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(source: Path, decision_path: Path, output: Path) -> dict[str, Any]:
    source = source.resolve()
    decision_path = decision_path.resolve()
    output = output.resolve()
    if output == source:
        raise ValueError("candidate output must not overwrite the source airframe")
    if "debug_4kg" not in source.name or source.name.startswith("4026"):
        raise ValueError("candidate builder only accepts the disposable 4 kg debug airframe")
    if "7p735" in output.name.lower() or output.name.startswith("4026"):
        raise ValueError("candidate output must not use a formal 7.735 kg airframe name")

    decision = json.loads(decision_path.read_text(encoding="utf-8"))
    if decision.get("decision") != "BOUNDED_CANDIDATE_REQUIRES_RETEST":
        raise ValueError("tuning decision does not authorize a bounded candidate")
    changes = decision.get("parameter_changes")
    if not isinstance(changes, list) or not changes:
        raise ValueError("tuning decision contains no parameter changes")

    by_parameter: dict[str, dict[str, Any]] = {}
    for change in changes:
        parameter = str(change.get("parameter", ""))
        old = change.get("old")
        candidate = change.get("candidate")
        relative = change.get("relative_change")
        if not parameter or parameter in by_parameter:
            raise ValueError(f"invalid or duplicate parameter {parameter!r}")
        if change.get("status") != "EXPERIMENTAL_NOT_APPLIED":
            raise ValueError(f"{parameter}: change is not an experimental candidate")
        if not all(isinstance(value, (int, float)) for value in (old, candidate, relative)):
            raise ValueError(f"{parameter}: old/candidate/relative values must be numeric")
        if not all(math.isfinite(float(value)) for value in (old, candidate, relative)):
            raise ValueError(f"{parameter}: non-finite value")
        calculated = (float(candidate) - float(old)) / float(old)
        if not math.isclose(calculated, float(relative), abs_tol=1.0e-9):
            raise ValueError(f"{parameter}: relative change does not match old/candidate")
        if abs(float(relative)) > 0.05 + 1.0e-12:
            raise ValueError(f"{parameter}: change exceeds the 5 percent iteration bound")
        by_parameter[parameter] = change

    replaced: set[str] = set()
    output_lines = []
    for line in source.read_text(encoding="utf-8").splitlines():
        match = PARAM_RE.match(line)
        if not match or match.group(2) not in by_parameter:
            output_lines.append(line)
            continue
        parameter = match.group(2)
        if parameter in replaced:
            raise ValueError(f"{parameter}: source airframe defines parameter more than once")
        old_in_source = float(match.group(4))
        expected_old = float(by_parameter[parameter]["old"])
        if not math.isclose(old_in_source, expected_old, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError(
                f"{parameter}: source value {old_in_source} differs from decision old {expected_old}"
            )
        candidate = float(by_parameter[parameter]["candidate"])
        output_lines.append(
            f"{match.group(1)}{parameter}{match.group(3)}{candidate:.9g}{match.group(5)}"
        )
        replaced.add(parameter)
    missing = sorted(set(by_parameter) - replaced)
    if missing:
        raise ValueError(f"source airframe is missing parameters: {missing}")

    header = [
        "# DISPOSABLE 4 KG TUNING CANDIDATE - NOT FORMAL PHYSICS",
        f"# source_sha256={sha256(source)}",
        f"# decision_sha256={sha256(decision_path)}",
        "# Rerun the identical identification protocol before accepting any value.",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(header + output_lines) + "\n", encoding="utf-8")
    return {
        "result": "PX4_4KG_TUNING_CANDIDATE_BUILT",
        "source": str(source),
        "source_sha256": sha256(source),
        "decision": str(decision_path),
        "decision_sha256": sha256(decision_path),
        "output": str(output),
        "output_sha256": sha256(output),
        "changes": changes,
        "formal_7p735_untouched": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("decision", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()
    result = build(args.source, args.decision, args.output)
    serialized = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.manifest:
        args.manifest.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
