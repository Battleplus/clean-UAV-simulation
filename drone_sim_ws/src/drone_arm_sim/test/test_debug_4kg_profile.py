import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from drone_arm_sim.gazebo_direct_motor_model import (
    actuator_command_to_thrust_n,
    thrust_to_actuator_command,
)
from drone_arm_sim.coupled_dynamics import CoupledArmDynamics
from drone_arm_sim.base1_wrench_reallocator import allocate_total_wrench
from drone_arm_sim.model_analysis import UrdfModel


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
FORMAL_URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
CONFIG = PACKAGE / "config/my_drone_v3_cad_debug_4kg.json"
REFERENCE = PACKAGE / "config/so101_motion_reference_4kg.json"
FORMAL_REFERENCE = PACKAGE / "config/so101_motion_reference.json"
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
        baro_noise = root.find(
            "./gazebo/sensor[@name='air_pressure_sensor']/air_pressure/pressure/noise/stddev"
        )
        self.assertIsNotNone(baro_noise)
        self.assertEqual(float(baro_noise.text), 0.2)

    def test_px4_hover_is_below_half_command(self):
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        hover = float(config["actuator_normalization"]["px4_hover_command"])
        self.assertGreater(hover, 0.40)
        self.assertLess(hover, 0.50)
        airframe = AIRFRAME.read_text(encoding="utf-8")
        self.assertIn(f"MPC_THR_HOVER {hover:.4f}", airframe)
        self.assertIn("MPC_THR_MIN 0.10", airframe)
        self.assertIn("MPC_Z_P 1.00", airframe)
        self.assertIn("MPC_Z_VEL_P_ACC 4.00", airframe)
        self.assertIn("MPC_Z_VEL_I_ACC 2.00", airframe)
        self.assertIn("MPC_Z_VEL_D_ACC 0.00", airframe)
        self.assertIn("MPC_Z_VEL_MAX_UP 0.25", airframe)
        self.assertIn("MPC_Z_VEL_MAX_DN 0.25", airframe)
        self.assertIn("MPC_XY_P 0.95", airframe)
        self.assertIn("MPC_XY_VEL_P_ACC 1.80", airframe)
        self.assertIn("MPC_XY_VEL_I_ACC 0.40", airframe)
        self.assertIn("MPC_XY_VEL_D_ACC 0.20", airframe)
        self.assertIn("CA_ROTOR0_KM -0.005000000", airframe)
        self.assertIn("CA_ROTOR2_KM 0.005000000", airframe)
        self.assertIn("EKF2_BARO_DELAY 0", airframe)
        self.assertIn("EKF2_GPS_CTRL 5", airframe)
        self.assertIn("EKF2_BARO_CTRL 1", airframe)
        self.assertIn("EKF2_EV_CTRL 12", airframe)
        self.assertIn("EKF2_EVA_NOISE 0.05", airframe)
        self.assertIn("EKF2_EVV_NOISE 0.05", airframe)
        self.assertNotIn("EKF2_EV_NOISE_MD", airframe)
        self.assertNotIn("EKF2_EVP_NOISE", airframe)
        self.assertIn("EKF2_HGT_REF 0", airframe)
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

    def test_isolated_world_starts_on_ground_level_contacts(self):
        root = ET.parse(WORLD).getroot()
        table = root.find("./world/model[@name='ground_plane']/link/collision[@name='tabletop_collision']")
        self.assertIsNotNone(table)
        table_pose = [float(v) for v in table.find("pose").text.split()]
        table_size = [float(v) for v in table.find("geometry/box/size").text.split()]
        self.assertAlmostEqual(table_pose[2] + table_size[2] / 2.0, 0.0)
        support = root.find("./world/model[@name='my_drone_bringup_landing_support']")
        self.assertIsNotNone(support)
        for link in support.findall("link"):
            size = [float(v) for v in link.find("collision/geometry/box/size").text.split()]
            pose = [float(v) for v in link.find("pose").text.split()]
            self.assertAlmostEqual(size[2], 0.02)
            self.assertAlmostEqual(pose[2] + size[2] / 2.0, 0.0)

    def test_debug_launcher_uses_gentle_manual_vertical_speed(self):
        launcher = (
            WORKSPACE / "scripts/wsl_start_ros2_dds_debug_4kg.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('PX4_WASD_VERTICAL_SPEED_M_S:-0.15', launcher)
        self.assertIn('SPAWN_Z="${SPAWN_Z:-0.289}"', launcher)
        self.assertIn('PX4_TOUCHDOWN_DISARM_HEIGHT_M:-0.10', launcher)
        self.assertIn('PX4_TOUCHDOWN_CONTACT_ALLOWANCE_M:-0.02', launcher)
        self.assertIn('PX4_TRUTH_HOLD_XY_D:-0.0', launcher)
        self.assertIn('BASE1_GRAVITY_TORQUE_GAIN="${BASE1_GRAVITY_TORQUE_GAIN:-1}"', launcher)
        self.assertIn(
            'BASE1_GRAVITY_TORQUE_LIMIT_NM:-1.35', launcher
        )
        self.assertIn(
            'BASE1_COMP_MAX_MOTOR_DELTA_N:-1.60', launcher
        )
        self.assertIn('BASE1_REACTION_TORQUE_GAIN="${BASE1_REACTION_TORQUE_GAIN:-1}"', launcher)
        self.assertIn('BASE1_REACTION_FORCE_GAIN="${BASE1_REACTION_FORCE_GAIN:-1}"', launcher)
        self.assertIn(
            'BASE1_POSITION_FEEDBACK_ENABLED="${BASE1_POSITION_FEEDBACK_ENABLED:-false}"',
            launcher,
        )
        self.assertIn('BASE1_ADAPTIVE_ENABLED="${BASE1_ADAPTIVE_ENABLED:-false}"', launcher)
        # The read-only estimator does not enable the JTC reaction-wrench
        # path, so the launcher must report the effective runtime state.
        self.assertIn(
            'BASE1_ARM_COMPENSATION_READY gravity=true dynamic_wrench_6d=false',
            launcher,
        )

    def test_vehicle_does_not_invent_landing_gear(self):
        root = ET.parse(URDF).getroot()
        base = root.find("./link[@name='base_link']")
        self.assertIsNotNone(base)
        visuals = [v.get("name", "") for v in base.findall("visual")]
        collisions = [c.get("name", "") for c in base.findall("collision")]
        self.assertEqual(sum("landing_leg" in name for name in visuals), 0)
        self.assertEqual(sum("landing_leg" in name for name in collisions), 0)
        self.assertIn("retracted_arm_ground_support_collision", collisions)
        self.assertIsNotNone(root.find("./link[@name='gripper_link']/collision"))
        self.assertIsNotNone(root.find("./link[@name='moving_jaw_link']/collision"))

    def test_retracted_arm_links_have_support_collisions(self):
        root = ET.parse(URDF).getroot()
        for link_name in (
            "arm_base_link", "shoulder_link", "upper_arm_link",
            "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_link",
        ):
            collisions = root.findall(f"./link[@name='{link_name}']/collision")
            self.assertTrue(
                any("support_collision" in c.get("name", "") for c in collisions),
                link_name,
            )

    def test_base1_arm_mount_is_connected_and_ground_pose_remains_folded(self):
        root = ET.parse(URDF).getroot()
        mount = root.find("./joint[@name='arm_mount']/origin")
        self.assertIsNotNone(mount)
        self.assertTrue(
            np.allclose(
                [float(value) for value in mount.get("xyz").split()],
                [0.0, 0.0, 0.0],
                atol=1.0e-9,
            )
        )
        self.assertTrue(
            np.allclose(
                [float(value) for value in mount.get("rpy").split()],
                [0.0, 0.0, 0.0],
                atol=1.0e-9,
            )
        )

        reference = json.loads(REFERENCE.read_text(encoding="utf-8"))
        names = [item["name"] for item in reference["joints"]]
        folded = dict(zip(names, reference["presets"]["retracted"]))
        formal_reference = json.loads(FORMAL_REFERENCE.read_text(encoding="utf-8"))
        formal_names = [item["name"] for item in formal_reference["joints"]]
        formal_folded = dict(
            zip(formal_names, formal_reference["presets"]["retracted"])
        )
        model = UrdfModel(URDF)
        formal_model = UrdfModel(FORMAL_URDF)
        end, active = model.forward_kinematics("moving_jaw_link", folded)
        formal_end, formal_active = formal_model.forward_kinematics(
            "moving_jaw_link", formal_folded
        )
        points = np.asarray([point for _, point, _ in active] + [end[:3, 3]])
        formal_points = np.asarray(
            [point for _, point, _ in formal_active] + [formal_end[:3, 3]]
        )
        # Re-indexing changes command coordinates only.  Ground geometry must
        # be the original folded CAD configuration, point for point.
        self.assertTrue(np.allclose(points, formal_points, atol=1.0e-8))
        self.assertLess(float(np.ptp(points[1:, 0])), 0.25)

        controls = root.find("./ros2_control")
        for name in names:
            initial = controls.find(
                f"./joint[@name='{name}']/state_interface[@name='position']"
                "/param[@name='initial_value']"
            )
            self.assertIsNotNone(initial)
            self.assertAlmostEqual(float(initial.text), folded[name], places=9)

        mass, center, inertia = model.mass_properties(folded)
        _, formal_center, _ = formal_model.mass_properties(formal_folded)
        self.assertAlmostEqual(mass, 4.0, places=9)
        # Uniform mass scaling and command re-indexing must preserve the formal
        # folded geometry and its CAD centre of mass exactly.
        self.assertTrue(np.allclose(center, formal_center, atol=1.0e-8))
        self.assertGreater(abs(float(center[0])), 0.002)
        self.assertLess(abs(float(center[0])), 0.02)
        self.assertLess(abs(float(center[1])), 0.02)
        self.assertTrue(np.all(np.linalg.eigvalsh(inertia) > 0.0))

        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        installation = config["arm_installation"]
        self.assertEqual(
            installation["profile"],
            "base1_cad_connected_folded_ground_nose_forward_straight_flight",
        )
        self.assertIn(
            "formal 7.735 kg CAD assembly and motion reference are unchanged",
            installation["status"],
        )
        self.assertAlmostEqual(installation["ground_spawn_z_m"], 0.289)

    def test_airborne_forward_pose_is_horizontal_with_zero_base_yaw(self):
        reference = json.loads(REFERENCE.read_text(encoding="utf-8"))
        names = [item["name"] for item in reference["joints"]]
        limits = {item["name"]: item for item in reference["joints"]}
        for preset in ("retracted", "flight_straight_forward"):
            for name, value in zip(names, reference["presets"][preset]):
                self.assertGreaterEqual(value, limits[name]["lower_rad"])
                self.assertLessEqual(value, limits[name]["upper_rad"])

        model = UrdfModel(URDF)
        straight = dict(
            zip(names, reference["presets"]["flight_straight_forward"])
        )
        end, active = model.forward_kinematics("moving_jaw_link", straight)
        points = np.asarray([point for _, point, _ in active] + [end[:3, 3]])[1:]
        # Every movable centre advances along the frozen ROS FLU nose (+X),
        # and shoulder_pan is identical at both endpoints (no base yaw).
        self.assertTrue(np.all(np.diff(points[:, 0]) >= -1.0e-9))
        self.assertGreater(float(np.ptp(points[:, 0])), 0.33)
        self.assertLess(float(np.ptp(points[:, 1])), 0.04)
        self.assertLess(float(np.ptp(points[:, 2])), 0.04)
        folded = dict(zip(names, reference["presets"]["retracted"]))
        folded_end, _ = model.forward_kinematics("moving_jaw_link", folded)
        self.assertGreater(end[0, 3] - folded_end[0, 3], 0.20)
        self.assertAlmostEqual(straight["shoulder_pan"], folded["shoulder_pan"], places=12)

    def test_formal_motion_reference_is_not_rewritten(self):
        formal = json.loads(FORMAL_REFERENCE.read_text(encoding="utf-8"))
        debug = json.loads(REFERENCE.read_text(encoding="utf-8"))
        self.assertNotIn("flight_straight_forward", formal["presets"])
        self.assertIn("flight_straight_forward", debug["presets"])
        self.assertEqual(
            formal["presets"]["retracted"],
            [0.0, 0.0, 0.0, 0.0, -1.57079632679, 0.0],
        )

    def test_key6_is_flight_gated_left_extension_and_return(self):
        keyboard = (WORKSPACE / "scripts/run_ros2_arm_keyboard.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("request_hover_and_wait_stable", keyboard)
        self.assertIn("run_preset flight_straight_forward", keyboard)
        self.assertIn('run_preset retracted "${return_duration}"', keyboard)
        self.assertIn(
            'ARM_KEY6_AIRBORNE_RETURN_DURATION_S:-120', keyboard
        )
        self.assertIn("MY_DRONE_FLIGHT_CONFIG", keyboard)
        self.assertIn("--flight-preflight", keyboard)
        self.assertIn('local direct_xy_ownership=false', keyboard)
        self.assertIn('local arming_state="${4:-}"', keyboard)
        self.assertIn("get_vehicle_arming_state", keyboard)
        self.assertIn("--vehicle-status-samples 1 --print-arming-state", keyboard)
        self.assertIn("ARM_VEHICLE_STATUS_TIMEOUT_S:-12", keyboard)
        self.assertIn('VEHICLE_ARMING_STATE state=1', keyboard)
        self.assertNotIn(
            "grep -Eq 'VEHICLE_ARMING_STATE state=[0-9]+'", keyboard
        )
        self.assertIn('profile_label="1.3kg candidate"', keyboard)
        self.assertIn('echo "Active profile: ${profile_label}"', keyboard)
        self.assertIn(
            "VISIBLE_DEMO_REFUSED: PX4 arming state is unavailable", keyboard
        )
        self.assertIn(
            'run_preset flight_straight_forward "${duration}" 0.02 "${arming_state}"',
            keyboard,
        )
        self.assertIn(
            'run_preset retracted "${return_duration}" 0.02 "${arming_state}"',
            keyboard,
        )
        self.assertNotIn(
            "timeout 3 ros2 topic echo --once /fmu/out/vehicle_status_v4",
            keyboard,
        )
        self.assertNotIn("vehicle_is_armed()", keyboard)
        self.assertIn(
            'ARM_DIRECT_XY_OWNERSHIP="${direct_xy_ownership}"', keyboard
        )
        self.assertIn("require_fresh_arm_runtime", keyboard)
        self.assertIn(
            "ARM_RUNTIME_UNAVAILABLE reason=joint_states_stale_or_gazebo_stopped",
            keyboard,
        )
        self.assertIn("ARM_RUNTIME_UNAVAILABLE reason=arm_controller_missing", keyboard)
        self.assertIn("KEY6_DEMO_FAILED: controller remains open", keyboard)
        self.assertIn("arm_keyboard.log", keyboard)
        self.assertNotIn("--distance \"${distance}\"", keyboard)

        sample_gate = (
            WORKSPACE / "scripts/wait_base1_ros_samples.py"
        ).read_text(encoding="utf-8")
        self.assertIn("self.latest_arming_state", sample_gate)
        self.assertIn('"--print-arming-state"', sample_gate)
        self.assertIn("VEHICLE_ARMING_STATE state=", sample_gate)

        candidate_launcher = (
            WORKSPACE / "scripts/start_candidate_1p3kg_joint_debug.ps1"
        ).read_text(encoding="utf-8")
        self.assertNotIn("Start-Sleep -Seconds 8", candidate_launcher)
        self.assertIn("wait_base1_ros_samples.py", candidate_launcher)
        self.assertIn(
            "--joint-samples 3 --vehicle-status-samples 1 --timeout 240",
            candidate_launcher,
        )

        flight_driver = (
            WORKSPACE / "scripts/test_ros2_dds_arm_flight_pty.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'arm_environment["SO101_MOTION_REFERENCE"]', flight_driver
        )
        self.assertIn("so101_motion_reference_4kg.json", flight_driver)
        self.assertIn('profile == "nose_forward_straight_4kg"', flight_driver)
        self.assertIn("ARM_NOSE_FORWARD_RETURN_DURATION_S", flight_driver)
        self.assertIn('arm_command.append("--flight-preflight")', flight_driver)

        controller = (
            WORKSPACE
            / "src/px4_ros2_control/px4_ros2_control/dds_wasd_control.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'os.environ.get("PX4_TRUTH_HOLD_XY_D", "0.0")', controller
        )

    def test_compensation_reference_is_the_folded_pose(self):
        reference = json.loads(REFERENCE.read_text(encoding="utf-8"))
        names = [item["name"] for item in reference["joints"]]
        dynamics = CoupledArmDynamics(URDF, REFERENCE, target_mass_kg=4.0)
        folded = dict(zip(names, reference["presets"]["retracted"]))
        straight = dict(
            zip(names, reference["presets"]["flight_straight_forward"])
        )
        folded_state = dynamics.state(folded)
        straight_state = dynamics.state(straight)
        self.assertLess(float(np.linalg.norm(folded_state.com_shift_m)), 1.0e-9)
        self.assertGreater(float(np.linalg.norm(straight_state.com_shift_m)), 0.015)

        # The new nose-forward pose creates roughly 1.28 N.m of pitch load.
        # Its production 4 kg bounds must realize that load without invoking
        # the residual fallback, otherwise compensation repeatedly disappears
        # and rebuilds during the return stroke.
        gravity_flu = np.asarray([0.0, 0.0, -4.0 * 9.80665])
        gravity_torque_flu = np.cross(straight_state.com_shift_m, gravity_flu)
        flu_to_frd = np.diag([1.0, -1.0, -1.0])
        compensation_frd = np.zeros(6)
        compensation_frd[3:] = -(flu_to_frd @ gravity_torque_flu)
        self.assertLess(float(np.linalg.norm(compensation_frd[3:])), 1.35)

        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        hover = float(config["actuator_normalization"]["px4_hover_command"])
        allocation = allocate_total_wrench(
            config,
            np.full(8, hover),
            compensation_frd,
            maximum_motor_delta_n=1.60,
        )
        self.assertTrue(allocation["success"])
        self.assertLess(allocation["residual_norm"], 1.0e-6)
        self.assertEqual(
            int(
                np.count_nonzero(
                    allocation["saturated_low"] | allocation["saturated_high"]
                )
            ),
            0,
        )
        commands = allocation["commands_motor_order"]
        self.assertGreater(float(np.min(commands)), 0.30)
        self.assertLess(float(np.max(commands)), 0.65)


if __name__ == "__main__":
    unittest.main()
