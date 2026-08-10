import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from drone_arm_sim.gazebo_direct_motor_model import (
    actuator_command_to_thrust_n,
    thrust_to_actuator_command,
)


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
CONFIG = PACKAGE / "config/my_drone_v3_cad_debug_4kg.json"
AIRFRAME = WORKSPACE / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
WORLD = PACKAGE / "worlds/flight_world_debug_4kg.sdf"


class Debug4kgProfileTest(unittest.TestCase):
    def test_mass_and_ideal_dynamics(self):
        root = ET.parse(URDF).getroot()
        masses = [
            float(mass.get("value"))
            for mass in root.findall("./link/inertial/mass")
        ]
        self.assertAlmostEqual(sum(masses), 4.0, places=9)
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(config["estimated_all_up_mass_kg"], 4.0)
        self.assertGreater(config["estimated_vertical_thrust_to_weight"], 2.0)
        self.assertFalse(config["battery_dynamics"]["enabled"])
        self.assertEqual(config["actuator_input_model"], "ideal_linear_thrust")
        self.assertEqual(config["actuator_normalization"]["rated_thrust_command"], 1.0)
        self.assertTrue(config["takeoff_support_release"]["enabled"])
        imu_rate = root.find("./gazebo/sensor[@name='imu_sensor']/update_rate")
        self.assertIsNotNone(imu_rate)
        self.assertEqual(float(imu_rate.text), 250.0)

    def test_px4_hover_is_below_half_command(self):
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        hover = float(config["actuator_normalization"]["px4_hover_command"])
        self.assertGreater(hover, 0.40)
        self.assertLess(hover, 0.50)
        airframe = AIRFRAME.read_text(encoding="utf-8")
        self.assertIn(f"MPC_THR_HOVER {hover:.4f}", airframe)
        self.assertIn("MPC_THR_MIN 0.10", airframe)
        self.assertIn("MPC_Z_P 0.35", airframe)
        self.assertIn("MPC_Z_VEL_P_ACC 2.20", airframe)
        self.assertIn("MPC_Z_VEL_D_ACC 0.20", airframe)
        self.assertIn("MPC_Z_VEL_MAX_UP 0.25", airframe)
        self.assertIn("MPC_Z_VEL_MAX_DN 0.25", airframe)
        self.assertIn("MPC_XY_P 2.20", airframe)
        self.assertIn("CA_ROTOR0_KM -0.001000000", airframe)
        self.assertIn("CA_ROTOR2_KM 0.001000000", airframe)
        self.assertIn("EKF2_HGT_REF 1", airframe)
        self.assertIn("EKF2_GPS_CTRL 7", airframe)
        self.assertIn("EKF2_BARO_CTRL 0", airframe)
        self.assertIn("EKF2_BARO_DELAY 20", airframe)
        self.assertIn("MC_ROLL_P 3.00", airframe)

    def test_ideal_linear_mapping_is_used_in_both_directions(self):
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        maximum = float(config["maximum_thrust_n"])
        for command in (0.0, 0.2, 0.4811252243, 0.75, 1.0):
            thrust = actuator_command_to_thrust_n(config, command)
            self.assertAlmostEqual(thrust, command * maximum, places=9)
            self.assertAlmostEqual(
                thrust_to_actuator_command(config, thrust), command, places=9
            )

    def test_isolated_world_uses_stable_bench_contacts(self):
        root = ET.parse(WORLD).getroot()
        support = root.find("./world/model[@name='my_drone_bringup_landing_support']")
        self.assertIsNotNone(support)
        for link in support.findall("link"):
            size = [float(v) for v in link.find("collision/geometry/box/size").text.split()]
            pose = [float(v) for v in link.find("pose").text.split()]
            self.assertAlmostEqual(size[2], 0.817)
            self.assertAlmostEqual(pose[2] + size[2] / 2.0, 0.817)

    def test_debug_launcher_uses_gentle_manual_vertical_speed(self):
        launcher = (
            WORKSPACE / "scripts/wsl_start_ros2_dds_debug_4kg.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('PX4_WASD_VERTICAL_SPEED_M_S:-0.15', launcher)


if __name__ == "__main__":
    unittest.main()
