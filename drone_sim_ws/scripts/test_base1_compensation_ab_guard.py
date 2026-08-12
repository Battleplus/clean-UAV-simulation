from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parent / "run_base1_compensation_ab.sh"


class Base1CompensationABGuardTest(unittest.TestCase):
    def test_runner_is_4kg_only_and_rejects_unplanned_gain(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("my_drone_v3_cad_debug_4kg.json", text)
        self.assertIn('gain must be exactly 0.05 or 0.10', text)
        self.assertNotIn("7p735", text.lower())
        self.assertNotIn("7.735", text)

    def test_both_sides_use_overlay_and_same_slow_profile(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("full_extend_slow_4kg", text)
        self.assertIn("BASE1_COMPENSATION_ENABLED=true", text)
        self.assertIn("activate_base1_wrench_reallocator_overlay.sh", text)
        self.assertIn("post_overlay_stability.log", text)
        self.assertIn("--horizontal 0.10 --vertical 0.08 --hold 10", text)

    def test_failed_side_aborts_pair_before_next_side(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("BASE1_COMP_AB_ABORT", text)
        self.assertIn("if (( flight_status != 0 )); then", text)
        self.assertNotIn("(( flight_status != 0 )) && overall=1", text)


if __name__ == "__main__":
    unittest.main()
