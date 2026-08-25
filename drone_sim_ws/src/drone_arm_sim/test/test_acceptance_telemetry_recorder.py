import ast
import json
from pathlib import Path
from types import MethodType, SimpleNamespace


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "drone_arm_sim/acceptance_telemetry_recorder.py"
)


def _callback_recorder():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    recorder_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "AcceptanceTelemetryRecorder"
    )
    callback = next(
        node
        for node in recorder_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_on_direct_xy_controller_intent"
    )
    module = ast.fix_missing_locations(ast.Module(body=[callback], type_ignores=[]))
    namespace = {"json": json, "String": object}
    exec(compile(module, str(SOURCE), "exec"), namespace)
    captured = []
    recorder = SimpleNamespace(_write=captured.append)
    recorder._on_direct_xy_controller_intent = MethodType(
        namespace["_on_direct_xy_controller_intent"], recorder
    )
    return recorder, captured


def test_recorder_subscribes_to_authoritative_controller_intent_topic():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"/my_drone/arm_direct_xy_controller_intent"' in source
    assert "self._on_direct_xy_controller_intent" in source


def test_recorder_safety_evidence_uses_reliable_latest_qos():
    source = SOURCE.read_text(encoding="utf-8")
    profile_start = source.index("latest_safety_level_qos = QoSProfile(")
    subscriptions_end = source.index(
        "# These are the actual ROS -> PX4 input messages", profile_start
    )
    safety_section = source[profile_start:subscriptions_end]

    assert "reliability=ReliabilityPolicy.RELIABLE" in safety_section
    assert "history=HistoryPolicy.KEEP_LAST" in safety_section
    assert "depth=1" in safety_section
    assert '"/my_drone/arm_direct_xy_guardian/state"' in safety_section
    assert '"/my_drone/arm_direct_xy_controller_intent"' in safety_section


def test_controller_intent_callback_records_decoded_authoritative_level():
    recorder, captured = _callback_recorder()
    payload = {
        "schema": "my_drone.arm-direct-xy-controller-intent.v1",
        "monotonic_s": 12.0,
        "controller_session_id": "controller-a",
        "ownership_epoch": 2,
        "intent_generation": 3,
        "state": "direct_xy",
        "motion_active": True,
        "mixed_setpoint_active": True,
        "force_requested": True,
    }

    recorder._on_direct_xy_controller_intent(
        SimpleNamespace(data=json.dumps(payload))
    )

    assert captured == [
        {"kind": "arm_direct_xy_controller_intent", "state": payload}
    ]


def test_controller_intent_callback_marks_invalid_json_without_guessing():
    recorder, captured = _callback_recorder()

    recorder._on_direct_xy_controller_intent(SimpleNamespace(data="{"))

    assert captured == [{"kind": "arm_direct_xy_controller_intent_invalid"}]
