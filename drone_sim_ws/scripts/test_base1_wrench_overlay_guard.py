"""Static guard for the Base 1-only compensation overlay launcher."""

from pathlib import Path
import unittest


WORKSPACE = Path(__file__).resolve().parents[1]
SCRIPT = WORKSPACE / "scripts/activate_base1_wrench_reallocator_overlay.sh"


class Base1WrenchOverlayGuardTest(unittest.TestCase):
    def test_overlay_is_4kg_only_and_defaults_off(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("my_drone_v3_cad_debug_4kg.json", text)
        self.assertIn('BASE1_COMPENSATION_ENABLED:-false', text)
        self.assertNotIn("7p735", text.lower())
        self.assertNotIn("7.735", text)

    def test_overlay_separates_raw_and_compensated_topics(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("/my_drone/command/motor_speed", text)
        self.assertIn("/my_drone/base1_compensated/command/motor_speed", text)
        self.assertIn("--arm-torque-feedforward-enabled false", text)
        self.assertIn("--arm-disturbance-observer-enabled false", text)


if __name__ == "__main__":
    unittest.main()
