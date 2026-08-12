"""Read-only audit of metadata embedded in the formal CAD propeller part.

Geometry and configuration checks already prove that all eight assembly
instances reuse one non-mirrored, effectively zero-pitch model.  This script
closes a separate loophole: a normal/reverse or CW/CCW designation hidden in
SolidWorks summary fields, custom properties, configuration metadata,
equations or feature names.

The source document is opened silent/read-only and SolidWorks is always
closed.  No CAD document is saved.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import pythoncom
import win32com.client


SW_DOC_PART = 1
SW_OPEN_SILENT = 1
SW_OPEN_READ_ONLY = 2

SUMMARY_FIELDS = {
    0: "title",
    1: "subject",
    2: "author",
    3: "keywords",
    4: "comment",
    5: "saved_by",
}

HANDEDNESS_TOKENS = (
    "cw",
    "ccw",
    "clockwise",
    "counterclockwise",
    "counter-clockwise",
    "normal",
    "reverse",
    "left hand",
    "right hand",
    "left-hand",
    "right-hand",
    "pitch",
    "正桨",
    "反桨",
    "左旋",
    "右旋",
    "螺距",
)


def call_or_value(obj: Any, name: str):
    value = getattr(obj, name)
    return value() if callable(value) else value


def safe_value(obj: Any, name: str, default=None):
    try:
        return call_or_value(obj, name)
    except Exception:
        return default


def string_value(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (tuple, list)):
        return " | ".join(string_value(item) for item in value if item is not None)
    return str(value)


def read_custom_properties(model: Any, configuration: str) -> dict[str, str]:
    manager = model.Extension.CustomPropertyManager(configuration)
    names = safe_value(manager, "GetNames", []) or []
    result: dict[str, str] = {}
    for raw_name in names:
        name = str(raw_name)
        value = ""
        # Get2 is widely available across SolidWorks versions.  Depending on
        # the generated COM wrapper it returns either a scalar or a tuple of
        # raw/resolved values; preserving both is preferable to guessing.
        try:
            value = string_value(manager.Get2(name))
        except Exception:
            try:
                value = string_value(manager.Get(name))
            except Exception as error:
                value = f"API_ERROR:{type(error).__name__}"
        result[name] = value
    return result


def read_features(model: Any) -> tuple[list[dict[str, str]], str]:
    features: list[dict[str, str]] = []
    first_feature_error = None
    try:
        feature = call_or_value(model, "FirstFeature")
    except Exception as error:
        feature = None
        first_feature_error = type(error).__name__
    seen = 0
    while feature is not None and seen < 10000:
        features.append(
            {
                "name": string_value(safe_value(feature, "Name")),
                "type": string_value(safe_value(feature, "GetTypeName2")),
            }
        )
        feature = safe_value(feature, "GetNextFeature")
        seen += 1
    if features:
        return features, "MODELDOC_FIRST_FEATURE"

    # Some late-bound SolidWorks installations do not expose FirstFeature,
    # while FeatureManager.GetFeatures still returns the complete tree.
    try:
        # FeatureManager is a COM dispatch property.  A Dispatch object may
        # report itself as callable, so call_or_value would incorrectly invoke
        # the manager object instead of using it.
        manager = getattr(model, "FeatureManager")
        raw_features = manager.GetFeatures(True) or []
        for item in raw_features:
            features.append(
                {
                    "name": string_value(safe_value(item, "Name")),
                    "type": string_value(safe_value(item, "GetTypeName2")),
                }
            )
        return features, "FEATURE_MANAGER_GET_FEATURES"
    except Exception as error:
        details = first_feature_error or "NO_FEATURES_RETURNED"
        return features, f"API_UNAVAILABLE:{details}/{type(error).__name__}"


def read_equations(model: Any) -> list[str]:
    manager = safe_value(model, "GetEquationMgr")
    if manager is None:
        return []
    count = safe_value(manager, "GetCount", 0) or 0
    equations: list[str] = []
    for index in range(int(count)):
        try:
            equations.append(string_value(manager.Equation(index)))
        except Exception:
            try:
                equations.append(string_value(manager.Equation[index]))
            except Exception as error:
                equations.append(f"API_ERROR:{type(error).__name__}")
    return equations


def inspect(part: Path, output: Path) -> dict[str, Any]:
    app = win32com.client.DispatchEx("SldWorks.Application")
    app.Visible = False
    app.CommandInProgress = True
    errors = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warnings = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    document = None
    try:
        document = app.OpenDoc6(
            str(part.resolve()),
            SW_DOC_PART,
            SW_OPEN_SILENT | SW_OPEN_READ_ONLY,
            "",
            errors,
            warnings,
        )
        if document is None:
            raise RuntimeError(f"OpenDoc6 failed: {errors.value}/{warnings.value}")

        configurations = [
            str(value)
            for value in (safe_value(document, "GetConfigurationNames", []) or [])
        ]
        summary = {}
        for index, name in SUMMARY_FIELDS.items():
            try:
                summary[name] = string_value(document.SummaryInfo(index))
            except Exception:
                summary[name] = ""

        configuration_metadata = []
        for name in configurations:
            configuration = document.GetConfigurationByName(name)
            configuration_metadata.append(
                {
                    "name": name,
                    "description": string_value(
                        safe_value(configuration, "Description")
                    ),
                    "comment": string_value(safe_value(configuration, "Comment")),
                    "custom_properties": read_custom_properties(document, name),
                }
            )

        features, feature_api_status = read_features(document)
        equations = read_equations(document)
        searchable = json.dumps(
            {
                "summary": summary,
                "document_custom_properties": read_custom_properties(document, ""),
                "configuration_metadata": configuration_metadata,
                "features": features,
                "equations": equations,
            },
            ensure_ascii=False,
        ).casefold()
        matched_tokens = sorted(
            token for token in HANDEDNESS_TOKENS if token.casefold() in searchable
        )
        mirror_features = [
            feature
            for feature in features
            if "mirror" in (feature["name"] + " " + feature["type"]).casefold()
            or "镜像" in feature["name"]
        ]

        result: dict[str, Any] = {
            "schema": 1,
            "source_part": str(part.resolve()),
            "source_part_sha256": hashlib.sha256(part.read_bytes()).hexdigest(),
            "open_errors": int(errors.value),
            "open_warnings": int(warnings.value),
            "summary": summary,
            "document_custom_properties": read_custom_properties(document, ""),
            "configuration_metadata": configuration_metadata,
            "equations": equations,
            "features": features,
            "feature_api_status": feature_api_status,
            "mirror_features": mirror_features,
            "handedness_tokens_found": matched_tokens,
        }
        if feature_api_status.startswith("API_UNAVAILABLE"):
            result["conclusion"] = "METADATA_AUDIT_INCOMPLETE"
        elif matched_tokens or mirror_features:
            result["conclusion"] = "HANDEDNESS_METADATA_REQUIRES_REVIEW"
        else:
            result["conclusion"] = "NO_HANDEDNESS_METADATA_FOUND"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return result
    finally:
        if document is not None:
            app.CloseDoc(string_value(safe_value(document, "GetTitle")))
        app.CommandInProgress = False
        app.ExitApp()


if __name__ == "__main__":
    project = Path(__file__).resolve().parents[4]
    payload = inspect(
        project / "零件" / "完整零件" / "f螺旋桨.SLDPRT",
        project
        / "drone_sim_ws"
        / "analysis"
        / "cad_direct"
        / "propeller_metadata_evidence.json",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
