"""Probe exact SolidWorks body mass-property API output without modifying CAD."""

from __future__ import annotations

import json
from pathlib import Path

import pythoncom
import win32com.client


SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def main() -> None:
    project = Path(__file__).resolve().parents[4]
    assembly = project / "零件" / "完整零件" / "组合无人机.SLDASM"
    output = project / "drone_sim_ws" / "analysis" / "cad_direct" / "mass_property_probe.json"
    app = win32com.client.DispatchEx("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    errors = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warnings = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    doc = None
    try:
        doc = app.OpenDoc6(
            str(assembly),
            SW_DOC_ASSEMBLY,
            SW_OPEN_SILENT | SW_OPEN_READ_ONLY,
            "",
            errors,
            warnings,
        )
        if doc is None:
            raise RuntimeError(f"OpenDoc6 failed: {errors.value}/{warnings.value}")
        doc.ResolveAllLightWeightComponents(False)
        records = []
        for component in doc.GetComponents(False) or []:
            if bool(value(component, "IsSuppressed")):
                continue
            bodies = component.GetBodies2(0) or []
            if not bodies:
                continue
            body_records = []
            for body in bodies:
                item = {"name": str(value(body, "Name") or "")}
                for density in (1000.0, 1100.0, 1300.0):
                    try:
                        raw = [float(x) for x in body.GetMassProperties(density)]
                        item[f"mass_properties_density_{int(density)}"] = raw
                    except Exception as exc:
                        item[f"mass_properties_density_{int(density)}_error"] = repr(exc)
                for api_name in ("GetVolume", "GetSurfaceArea"):
                    try:
                        item[api_name] = float(value(body, api_name))
                    except Exception as exc:
                        item[api_name + "_error"] = repr(exc)
                body_records.append(item)
            records.append(
                {
                    "instance": str(value(component, "Name2")),
                    "path": str(value(component, "GetPathName") or ""),
                    "transform": [float(x) for x in component.Transform2.ArrayData],
                    "bodies": body_records,
                }
            )
            if len(records) >= 20:
                break
        output.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print(output)
        for record in records:
            lengths = [
                len(body.get("mass_properties_density_1000", []))
                for body in record["bodies"]
            ]
            print(record["instance"], lengths)
    finally:
        if doc is not None:
            app.CloseDoc(value(doc, "GetTitle"))
        app.CommandInProgress = False
        app.ExitApp()


if __name__ == "__main__":
    main()
