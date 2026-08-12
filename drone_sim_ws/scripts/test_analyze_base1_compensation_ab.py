import json
from pathlib import Path
import tempfile
import unittest

from analyze_base1_compensation_ab import compare, parse_reallocator_log


def flight_log(drift: float, altitude: float, max_tilt: float, rms_tilt: float) -> str:
    return f"""
STATE arm=1 nav=4 NED=(0.0,0.0,0.2)
OFFBOARD mode and ARM commands sent
STATE arm=2 nav=14 NED=(0.0,0.0,-1.0)
ARM_FLIGHT_CYCLE_METRICS label=demo_extended
ARM_FLIGHT_CYCLE_METRICS label=retracted
ARM_FLIGHT_METRICS horizontal_drift_m={drift} altitude_span_m={altitude} samples=100 max_truth_tilt_deg={max_tilt} rms_truth_tilt_deg={rms_tilt} max_arm_torque_nm=0.05 motor_saturation_rate=0 motor_saturation_samples=0/100
PX4 command ack: command=21 result=0
LANDING_DISARMED_CONFIRMED
DDS_ARM_FLIGHT_PASS
"""


class Base1CompensationABTest(unittest.TestCase):
    def test_reallocator_runtime_requires_nonzero_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "runtime.log"
            state = {
                "compensation_wrench_frd": [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                "residual_norm": 1e-6,
                "saturated": 0,
                "flight_allowed": True,
                "source_fresh": True,
                "headroom_ok": True,
            }
            path.write_text(MARKER + json.dumps(state) + "\n", encoding="utf-8")
            report = parse_reallocator_log(path)
            self.assertTrue(report["runtime_proven_active"])
            self.assertEqual(report["active_state_count"], 1)

    def test_pair_is_accepted_only_when_all_metrics_improve(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            off = root / "off.log"
            on = root / "on.log"
            runtime = root / "runtime.log"
            off.write_text(flight_log(0.10, 0.20, 1.0, 0.5), encoding="utf-8")
            on.write_text(flight_log(0.08, 0.18, 0.9, 0.4), encoding="utf-8")
            state = {
                "compensation_wrench_frd": [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                "residual_norm": 1e-6,
                "saturated": 0,
                "flight_allowed": True,
                "source_fresh": True,
                "headroom_ok": True,
            }
            runtime.write_text(MARKER + json.dumps(state) + "\n", encoding="utf-8")
            report = compare(off, on, runtime, "gravity_torque", 0.05)
            self.assertTrue(report["candidate_accepted_for_repeat"])
            on.write_text(flight_log(0.08, 0.22, 0.9, 0.4), encoding="utf-8")
            report = compare(off, on, runtime, "gravity_torque", 0.05)
            self.assertFalse(report["candidate_accepted_for_repeat"])


MARKER = "BASE1_COMPENSATION_STATE "


if __name__ == "__main__":
    unittest.main()
