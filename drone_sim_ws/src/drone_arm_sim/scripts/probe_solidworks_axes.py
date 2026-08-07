"""Read cylindrical-face axes from the exact SolidWorks CAD assembly.

This is a read-only diagnostic used before selecting the formal motor and
joint axes.  It records every cylindrical face on motor and propeller parts,
including enough geometric evidence to identify coaxial face groups.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pythoncom
import win32com.client


SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2


def call_or_value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def transform_point(data, point):
    scale = float(data[12])
    x, y, z = map(float, point)
    return [
        scale * (data[0] * x + data[3] * y + data[6] * z) + data[9],
        scale * (data[1] * x + data[4] * y + data[7] * z) + data[10],
        scale * (data[2] * x + data[5] * y + data[8] * z) + data[11],
    ]


def transform_direction(data, direction):
    x, y, z = map(float, direction)
    result = [
        data[0] * x + data[3] * y + data[6] * z,
        data[1] * x + data[4] * y + data[7] * z,
        data[2] * x + data[5] * y + data[8] * z,
    ]
    norm = math.sqrt(sum(value * value for value in result))
    return [value / norm for value in result]


def face_area(face):
    try:
        return float(call_or_value(face, "GetArea"))
    except Exception:
        return None


def extract(assembly: Path, output: Path):
    app = win32com.client.gencache.EnsureDispatch("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    errors = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warnings = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    document = None
    try:
        document = app.OpenDoc6(
            str(assembly.resolve()),
            SW_DOC_ASSEMBLY,
            SW_OPEN_SILENT | SW_OPEN_READ_ONLY,
            "",
            errors,
            warnings,
        )
        if document is None:
            raise RuntimeError(f"OpenDoc6 failed: {errors.value}/{warnings.value}")
        document.ResolveAllLightWeightComponents(False)
        records = []
        for component in document.GetComponents(False) or []:
            path = str(call_or_value(component, "GetPathName") or "")
            filename = Path(path).name.casefold()
            if filename not in {"f电机.sldprt", "f螺旋桨.sldprt"}:
                continue
            transform = component.Transform2
            if transform is None:
                continue
            matrix = [float(value) for value in transform.ArrayData]
            cylinders = []
            bodies = component.GetBodies2(0) or []
            for body_index, body in enumerate(bodies):
                for face_index, face in enumerate(body.GetFaces() or []):
                    surface = face.GetSurface()
                    if surface is None or not bool(call_or_value(surface, "IsCylinder")):
                        continue
                    params = [float(value) for value in surface.CylinderParams]
                    cylinders.append({
                        "body_index": body_index,
                        "face_index": face_index,
                        "local_origin_m": params[0:3],
                        "local_axis": params[3:6],
                        "radius_m": params[6],
                        "area_m2": face_area(face),
                        "assembly_origin_m": transform_point(matrix, params[0:3]),
                        "assembly_axis": transform_direction(matrix, params[3:6]),
                    })
            records.append({
                "instance": str(call_or_value(component, "Name2")),
                "path": path,
                "cylinders": cylinders,
            })
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print(output)
        for record in records:
            print(record["instance"], len(record["cylinders"]))
    finally:
        if document is not None:
            app.CloseDoc(call_or_value(document, "GetTitle"))
        app.CommandInProgress = False
        app.ExitApp()


if __name__ == "__main__":
    project = Path(__file__).resolve().parents[4]
    extract(
        project / "零件" / "完整零件" / "组合无人机.SLDASM",
        project / "drone_sim_ws" / "analysis" / "cad_direct" / "axis_face_probe.json",
    )
