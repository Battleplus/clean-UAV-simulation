"""Export URDF source meshes from the user-selected SolidWorks CAD tree.

The default paths are derived from this repository so Chinese filesystem names
never cross a shell argument boundary.  Source documents are opened read-only
and are never saved.  Exports are written only under meshes/my_drone_v2.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys

import pythoncom
import win32com.client


SW_DOC_PART = 1
SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2
SW_SAVE_SILENT = 1


def value(obj, name):
    result = getattr(obj, name)
    return result() if callable(result) else result


def merge_binary_stls(inputs: list[Path], output: Path) -> None:
    triangle_blocks = []
    triangle_count = 0
    for path in inputs:
        data = path.read_bytes()
        if len(data) < 84:
            raise ValueError(f"Invalid binary STL: {path}")
        count = struct.unpack_from("<I", data, 80)[0]
        expected = 84 + count * 50
        if len(data) != expected:
            raise ValueError(f"Only binary STL is supported: {path}")
        triangle_count += count
        triangle_blocks.append(data[84:])
    header = b"my_drone_v2 merged from SolidWorks component STL"[:80]
    output.write_bytes(
        header.ljust(80, b"\0")
        + struct.pack("<I", triangle_count)
        + b"".join(triangle_blocks)
    )


def save_as_stl(app, source: Path, document_type: int, output: Path) -> None:
    errors = win32com.client.VARIANT(
        pythoncom.VT_BYREF | pythoncom.VT_I4, 0
    )
    warnings = win32com.client.VARIANT(
        pythoncom.VT_BYREF | pythoncom.VT_I4, 0
    )
    document = app.OpenDoc6(
        str(source),
        document_type,
        SW_OPEN_SILENT | SW_OPEN_READ_ONLY,
        "",
        errors,
        warnings,
    )
    if document is None:
        raise RuntimeError(
            f"Could not open {source}: errors={errors.value}, "
            f"warnings={warnings.value}"
        )
    try:
        if document_type == SW_DOC_ASSEMBLY:
            document.ResolveAllLightWeightComponents(False)
        output.parent.mkdir(parents=True, exist_ok=True)
        component_pattern = f"{output.stem} - *.STL"
        for stale in output.parent.glob(component_pattern):
            stale.unlink()
        succeeded = document.SaveAs3(str(output), 0, SW_SAVE_SILENT)
        component_outputs = sorted(output.parent.glob(component_pattern))
        if not output.exists() and component_outputs:
            merge_binary_stls(component_outputs, output)
            succeeded = True
        if not succeeded or not output.exists():
            raise RuntimeError(
                f"STL export failed for {source}"
            )
        print(
            f"{source.name} -> {output} ({output.stat().st_size} bytes, "
            f"component_files={len(component_outputs)})"
        )
    finally:
        app.CloseDoc(value(document, "GetTitle"))


def export(output_root: Path) -> None:
    project_root = Path(__file__).resolve().parents[4]
    cad = project_root / "零件" / "完整零件"
    # SolidWorks' current assembly-STL option emits one file per leaf
    # component.  Those outputs already provide the separate motor, propeller,
    # frame, and SO101 meshes needed by later URDF stages; merging them also
    # produces the first complete-assembly visual.
    sources = (
        ("full_assembly.stl", cad / "组合无人机.SLDASM", SW_DOC_ASSEMBLY),
    )
    missing = [str(path) for _, path, _ in sources if not path.exists()]
    if missing:
        raise FileNotFoundError(f"Missing CAD source files: {missing}")

    app = win32com.client.DispatchEx("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    try:
        for filename, source, document_type in sources:
            save_as_stl(app, source, document_type, output_root / filename)
    finally:
        app.CommandInProgress = False
        app.ExitApp()


def main() -> None:
    project_root = Path(__file__).resolve().parents[4]
    default_output = (
        project_root
        / "drone_sim_ws"
        / "src"
        / "drone_arm_sim"
        / "meshes"
        / "my_drone_v2"
        / "visual"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=default_output)
    args = parser.parse_args()
    export(args.output_root.resolve())


if __name__ == "__main__":
    if sys.platform != "win32":
        raise SystemExit("Run this script with Windows Python and SolidWorks installed")
    main()
