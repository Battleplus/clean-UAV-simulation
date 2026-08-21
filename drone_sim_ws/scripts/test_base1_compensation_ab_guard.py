from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parent / "run_base1_compensation_ab.sh"
FLIGHT_DRIVER = Path(__file__).resolve().parent / "test_ros2_dds_arm_flight_pty.py"


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

    def test_px4_status_timeout_aborts_driver_without_waiting_full_timeout(self):
        text = FLIGHT_DRIVER.read_text(encoding="utf-8")
        self.assertIn("OFFBOARD_STREAM_STOPPED ", text)
        self.assertIn("ARM_FLIGHT_CONTROLLER_STREAM_STOPPED", text)


if __name__ == "__main__":
    unittest.main()
