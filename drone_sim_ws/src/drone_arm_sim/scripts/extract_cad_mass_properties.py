"""Extract provisional physical properties from the exact SolidWorks assembly.

The CAD files are opened read-only.  Every resolved solid body is evaluated
with IBody2.GetMassProperties at the assigned provisional density.  The
assembly origin remains the immutable CAD reference; a COM-centred ROS FLU
frame is additionally reported for dynamics and PX4 allocation.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pythoncom
import win32com.client


SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2
STRUCTURE_DENSITY = 1100.0  # kg/m^3 = 1.1 g/cm^3
COMPOSITE_DENSITY = 1300.0  # kg/m^3 = 1.3 g/cm^3


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def repair_utf8_mojibake(text: str) -> str:
    try:
        return text.encode("gbk").decode("utf-8")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return text


def transform_parts(data):
    d = np.asarray(data, dtype=float)
    scale = float(d[12])
    rotation = np.asarray(
        [[d[0], d[3], d[6]], [d[1], d[4], d[7]], [d[2], d[5], d[8]]]
    )
    translation = d[9:12]
    return rotation, translation, scale


def inertia_from_raw(raw):
    # SolidWorks IBody2.GetMassProperties: Ixx,Iyy,Izz,Ixy,Izx,Iyz.
    return np.asarray(
        [
            [raw[6], raw[9], raw[10]],
            [raw[9], raw[7], raw[11]],
            [raw[10], raw[11], raw[8]],
        ],
        dtype=float,
    )


def density_for(instance: str, path: str):
    text = (instance + " " + path).casefold()
    structure_tokens = (
        "base",
        "机架",
        "支架",
        "holder",
        "mounting_plate",
        "under_arm",
        "upper_arm",
        "rotation_pitch",
        "moving_jaw",
        "wrist_roll",
        "连杆",
        "工具端",
        "降落位",
    )
    if any(token.casefold() in text for token in structure_tokens):
        return STRUCTURE_DENSITY, "structure_density_1.1_g_cm3"
    return COMPOSITE_DENSITY, "provisional_composite_density_1.3_g_cm3"


def transform_point(rotation, translation, scale, point):
    return scale * (rotation @ np.asarray(point, dtype=float)) + translation


def main() -> None:
    project = Path(__file__).resolve().parents[4]
    assembly = project / "零件" / "完整零件" / "组合无人机.SLDASM"
    output = project / "drone_sim_ws" / "analysis" / "cad_direct" / "cad_mass_properties.json"
    app = win32com.client.DispatchEx("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    errors = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warnings = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    doc = None
    try:
        doc = app.OpenDoc6(
            str(assembly), SW_DOC_ASSEMBLY,
            SW_OPEN_SILENT | SW_OPEN_READ_ONLY, "", errors, warnings
        )
        if doc is None:
            raise RuntimeError(f"OpenDoc6 failed: {errors.value}/{warnings.value}")
        doc.ResolveAllLightWeightComponents(False)
        records = []
        total_mass = 0.0
        weighted_com = np.zeros(3)
        for component in doc.GetComponents(False) or []:
            if bool(value(component, "IsSuppressed")):
                continue
            transform = component.Transform2
            bodies = component.GetBodies2(0) or []
            if transform is None or not bodies:
                continue
            instance_raw = str(value(component, "Name2") or "")
            path_raw = str(value(component, "GetPathName") or "")
            instance = repair_utf8_mojibake(instance_raw)
            path = repair_utf8_mojibake(path_raw)
            density, density_source = density_for(instance, path)
            rotation, translation, scale = transform_parts(transform.ArrayData)
            body_records = []
            for body_index, body in enumerate(bodies):
                raw = [float(x) for x in body.GetMassProperties(density)]
                if len(raw) != 12 or raw[5] <= 0.0:
                    continue
                com_cad = transform_point(rotation, translation, scale, raw[0:3])
                inertia_cad_com = rotation @ inertia_from_raw(raw) @ rotation.T
                mass = raw[5]
                total_mass += mass
                weighted_com += mass * com_cad
                body_records.append(
                    {
                        "body_index": body_index,
                        "body_name": repair_utf8_mojibake(str(value(body, "Name") or "")),
                        "density_kg_m3": density,
                        "density_source": density_source,
                        "volume_m3": raw[3],
                        "surface_area_m2": raw[4],
                        "mass_kg": mass,
                        "local_com_m": raw[0:3],
                        "cad_assembly_com_m": com_cad.tolist(),
                        "cad_inertia_at_com_kg_m2": inertia_cad_com.tolist(),
                    }
                )
            if body_records:
                records.append(
                    {
                        "instance": instance,
                        "source_path": path,
                        "density_kg_m3": density,
                        "density_source": density_source,
                        "bodies": body_records,
                    }
                )
        if total_mass <= 0.0:
            raise RuntimeError("No positive-mass CAD bodies found")
        cad_com = weighted_com / total_mass
        inertia_cad = np.zeros((3, 3))
        for component in records:
            for body in component["bodies"]:
                mass = body["mass_kg"]
                com = np.asarray(body["cad_assembly_com_m"])
                delta = com - cad_com
                inertia_cad += np.asarray(body["cad_inertia_at_com_kg_m2"])
                inertia_cad += mass * (
                    float(delta @ delta) * np.eye(3) - np.outer(delta, delta)
                )
        cad_to_flu = np.asarray([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
        inertia_flu = cad_to_flu @ inertia_cad @ cad_to_flu.T
        density_counts = {}
        for record in records:
            density_counts[record["density_source"]] = density_counts.get(record["density_source"], 0) + len(record["bodies"])
        payload = {
            "schema": 1,
            "source_assembly": str(assembly),
            "source_api": "SolidWorks IBody2.GetMassProperties(density)",
            "cad_open_mode": "silent + read-only; no save",
            "cad_assembly_frame": "immutable source geometry reference",
            "density_rules": {
                "structure_kg_m3": STRUCTURE_DENSITY,
                "other_composite_kg_m3": COMPOSITE_DENSITY,
                "status": "provisional until motor/propeller/servo specifications and whole-aircraft weighing are supplied",
            },
            "component_records": len(records),
            "solid_body_records": sum(len(x["bodies"]) for x in records),
            "density_assignment_counts": density_counts,
            "estimated_total_mass_kg": total_mass,
            "estimated_cad_com_m": cad_com.tolist(),
            "estimated_ros_flu_com_from_cad_origin_m": (cad_to_flu @ cad_com).tolist(),
            "estimated_cad_inertia_at_com_kg_m2": inertia_cad.tolist(),
            "estimated_ros_flu_inertia_at_com_kg_m2": inertia_flu.tolist(),
            "components": records,
        }
        output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(output)
        print(f"components={len(records)} bodies={payload['solid_body_records']}")
        print(f"estimated_total_mass_kg={total_mass:.9f}")
        print(f"estimated_cad_com_m={cad_com.tolist()}")
        print(f"estimated_ros_flu_inertia_at_com_kg_m2={inertia_flu.tolist()}")
    finally:
        if doc is not None:
            app.CloseDoc(value(doc, "GetTitle"))
        app.CommandInProgress = False
        app.ExitApp()


if __name__ == "__main__":
    main()
