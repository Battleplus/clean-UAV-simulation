"""Extract a read-only SolidWorks assembly component manifest for URDF work.

This script is intentionally Windows-only.  It opens the exact assembly path
through the SolidWorks COM API, fully resolves components, and records every
component-instance transform.  Stored references from an older directory are
remapped by basename only when a byte-for-byte source file exists below the
user-selected CAD root.  No SolidWorks document is saved or modified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import pythoncom
import win32com.client


SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2


def value(obj, name):
    result = getattr(obj, name)
    return result() if callable(result) else result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_index(root: Path) -> dict[str, list[Path]]:
    result: dict[str, list[Path]] = {}
    for path in root.rglob("*"):
        if path.is_file():
            result.setdefault(path.name.casefold(), []).append(path)
    return result


def resolve_to_cad_root(stored_path: str, index: dict[str, list[Path]]) -> Path | None:
    candidates = index.get(Path(stored_path).name.casefold(), [])
    if len(candidates) == 1:
        return candidates[0]
    return None


def transform_data(component) -> list[float] | None:
    transform = component.Transform2
    if transform is None:
        return None
    return [float(item) for item in transform.ArrayData]


def extract(assembly: Path, cad_root: Path, output: Path) -> None:
    assembly = assembly.resolve()
    cad_root = cad_root.resolve()
    if cad_root not in assembly.parents:
        raise ValueError(f"Assembly must be below CAD root: {cad_root}")

    index = source_index(cad_root)
    app = win32com.client.DispatchEx("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    errors = win32com.client.VARIANT(
        pythoncom.VT_BYREF | pythoncom.VT_I4, 0
    )
    warnings = win32com.client.VARIANT(
        pythoncom.VT_BYREF | pythoncom.VT_I4, 0
    )
    document = None
    try:
        document = app.OpenDoc6(
            str(assembly),
            SW_DOC_ASSEMBLY,
            SW_OPEN_SILENT | SW_OPEN_READ_ONLY,
            "",
            errors,
            warnings,
        )
        if document is None:
            raise RuntimeError(
                f"SolidWorks OpenDoc6 failed: errors={errors.value}, "
                f"warnings={warnings.value}"
            )
        document.ResolveAllLightWeightComponents(False)
        components = document.GetComponents(False) or []
        records = []
        unresolved = []
        for component in components:
            stored_path = str(value(component, "GetPathName") or "")
            resolved = resolve_to_cad_root(stored_path, index)
            record = {
                "instance": str(value(component, "Name2")),
                "stored_path": stored_path,
                "source_path": str(resolved) if resolved else None,
                "suppressed": bool(value(component, "IsSuppressed")),
                "transform_array": transform_data(component),
            }
            if resolved is not None:
                record["source_sha256"] = sha256(resolved)
            elif stored_path:
                unresolved.append(stored_path)
            records.append(record)

        payload = {
            "schema": 1,
            "assembly": str(assembly),
            "assembly_sha256": sha256(assembly),
            "cad_root": str(cad_root),
            "solidworks_open_errors": int(errors.value),
            "solidworks_open_warnings": int(warnings.value),
            "component_count": len(records),
            "unresolved_stored_paths": sorted(set(unresolved)),
            "components": records,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(output)
        print(f"components={len(records)} unresolved={len(set(unresolved))}")
    finally:
        if document is not None:
            app.CloseDoc(value(document, "GetTitle"))
        app.CommandInProgress = False
        app.ExitApp()


def main() -> None:
    project_root = Path(__file__).resolve().parents[4]
    default_cad_root = project_root / "零件"
    default_assembly = default_cad_root / "完整零件" / "组合无人机.SLDASM"
    default_output = (
        project_root
        / "drone_sim_ws"
        / "analysis"
        / "cad_direct"
        / "assembly_manifest.json"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", type=Path, default=default_assembly)
    parser.add_argument("--cad-root", type=Path, default=default_cad_root)
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()
    extract(args.assembly, args.cad_root, args.output)


if __name__ == "__main__":
    if sys.platform != "win32":
        raise SystemExit("Run this script with Windows Python and SolidWorks installed")
    main()
