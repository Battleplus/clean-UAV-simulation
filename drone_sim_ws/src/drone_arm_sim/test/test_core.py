"""Regression tests for the my_drone simulation artifacts."""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.flight_control_demo import simulate
from drone_arm_sim.floating_base_reaction import reaction_twist
from drone_arm_sim.gazebo_wrench_controller import compute_wrench_enu
from drone_arm_sim.gazebo_direct_motor_model import (
    FRD_TO_FLU,
    battery_step,
    command_to_current_a,
    command_to_thrust_n,
    direct_wrench_flu,
    delayed_command_step,
    environment_wrench_world,
    first_order_motor_step,
    ground_effect_thrust_scale,
)
from drone_arm_sim.inverse_kinematics import solve_ik
from drone_arm_sim.gazebo_sensor_delay import message_stamp_seconds
from drone_arm_sim.model_analysis import UrdfModel, _rpy_matrix
from scripts.generate_cad_px4_airframe import _thrust_to_command


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
CONTROLLED_URDF = PACKAGE / "urdf" / "drone_with_arm_controlled.urdf"
OCTOROTOR_URDF = PACKAGE / "urdf" / "my_drone_octorotor_example.urdf"
CONFIG_PATH = PACKAGE / "config" / "octorotor_example.json"
MEASURED_CONFIG_PATH = PACKAGE / "config" / "my_drone_measured_mounts.json"
MEASURED_FLIGHT_URDF = (
    PACKAGE / "urdf" / "drone_with_arm_measured_flight_test.urdf"
)
MEASURED_PHYSICAL_URDF = (
    PACKAGE / "urdf" / "drone_with_arm_measured_physical_estimate.urdf"
)
CAD_V2_URDF = PACKAGE / "urdf" / "my_drone_v2" / "my_drone_cad_dynamic.urdf"
CAD_V2_CONFIG_PATH = PACKAGE / "config" / "my_drone_v2_cad.json"
CAD_V3_CONFIG_PATH = PACKAGE / "config" / "my_drone_v3_cad_physical.json"
CAD_V3_FORMAL_URDF = PACKAGE / "urdf" / "my_drone_v3" / "my_drone_cad_formal_dynamic.urdf"
CAD_V3_FORMAL_REPORT = PACKAGE / "config" / "my_drone_v3_cad_formal_urdf.json"
CAD_V3_FLIGHT_CONFIG_PATH = (
    PACKAGE / "config" / "my_drone_v3_cad_7p735_flight.json"
)
CAD_V3_AIRFRAME_PATH = (
    WORKSPACE / "px4" / "airframes" / "4026_gz_my_drone_octorotor_7p735"
)
CAD_MANIFEST_PATH = WORKSPACE / "analysis" / "cad_direct" / "assembly_manifest.json"
EXTRACTED_MOUNTS_PATH = (
    WORKSPACE / "analysis" / "motor_geometry" / "extracted_mounts.json"
)
AIRFRAME_PATH = (
    WORKSPACE / "px4" / "airframes" / "4015_gz_my_drone_octorotor"
)


class CoreRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = UrdfModel(CONTROLLED_URDF)
        cls.config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    def test_mass_properties_are_physical(self):
        mass, center, inertia = self.model.mass_properties({})
        self.assertAlmostEqual(mass, 2.632006, places=6)
        self.assertTrue(np.all(np.isfinite(center)))
        np.testing.assert_allclose(inertia, inertia.T, atol=1e-12)
        self.assertGreater(float(np.min(np.linalg.eigvalsh(inertia))), 0.0)

    def test_measured_static_thrust_interpolation_and_rated_cap(self):
        config = json.loads(CAD_V3_CONFIG_PATH.read_text(encoding="utf-8"))
        self.assertAlmostEqual(command_to_thrust_n(config, 0.50), 5.34462425)
        self.assertAlmostEqual(command_to_thrust_n(config, 0.90), 11.76798)
        midpoint = 0.5 * (5.34462425 + 5.5113373)
        self.assertAlmostEqual(command_to_thrust_n(config, 0.525), midpoint)

    def test_cad_flight_reaction_torque_signs_match_cw_ccw(self):
        config = json.loads(
            (PACKAGE / "config" / "my_drone_v2_cad_flight_pitch_corrected.json")
            .read_text(encoding="utf-8")
        )
        self.assertAlmostEqual(config["reaction_moment_ratio_m"], 0.001)
        self.assertEqual(
            config["reaction_moment_estimate"]["status"],
            "enabled as a clean single-bridge closed-loop estimate",
        )
        for rotor in config["rotors"]:
            expected = -1 if rotor["turning_direction"] == "CW" else 1
            self.assertEqual(rotor["direction"], expected)
        self.assertEqual(int(np.linalg.matrix_rank(allocation_matrix(config))), 6)

    def test_first_order_motor_rise_and_fall(self):
        config = {"motor_dynamics": {
            "rise_time_constant_s": 0.035,
            "fall_time_constant_s": 0.070,
        }}
        risen = first_order_motor_step(
            np.zeros(2), np.ones(2), 0.035, config
        )
        np.testing.assert_allclose(risen, np.full(2, 1.0 - np.exp(-1.0)))
        fallen = first_order_motor_step(
            np.ones(2), np.zeros(2), 0.070, config
        )
        np.testing.assert_allclose(fallen, np.full(2, np.exp(-1.0)))
        np.testing.assert_allclose(
            first_order_motor_step(np.zeros(2), np.ones(2), 1.0, {}),
            np.ones(2),
        )

    def test_environment_drag_opposes_relative_airflow(self):
        config = {"environment_dynamics": {
            "air_density_kg_m3": 1.2,
            "quadratic_drag_area_cd_m2_body_flu": [0.1, 0.2, 0.3],
            "angular_damping_n_m_per_rad_s_body_flu": [0.01, 0.02, 0.03],
            "wind_velocity_world_enu_m_s": [1.0, 0.0, 0.0],
        }}
        force, torque = environment_wrench_world(
            config, np.eye(3), np.array([3.0, -2.0, 0.0]),
            np.array([1.0, -2.0, 0.5]),
        )
        np.testing.assert_allclose(force, [-0.24, 0.48, 0.0])
        np.testing.assert_allclose(torque, [-0.01, 0.04, -0.015])
        zero_force, zero_torque = environment_wrench_world(
            {}, np.eye(3), np.ones(3), np.ones(3)
        )
        np.testing.assert_allclose(zero_force, np.zeros(3))
        np.testing.assert_allclose(zero_torque, np.zeros(3))

    def test_ground_effect_is_bounded_and_decays_with_height(self):
        config = {"environment_dynamics": {"ground_effect": {
            "enabled": True, "maximum_thrust_gain": 0.08,
            "decay_height_m": 0.25,
        }}}
        self.assertAlmostEqual(ground_effect_thrust_scale(config, 0.0), 1.08)
        self.assertGreater(ground_effect_thrust_scale(config, 0.25), 1.0)
        self.assertLess(ground_effect_thrust_scale(config, 0.25), 1.08)
        self.assertAlmostEqual(ground_effect_thrust_scale({}, 0.0), 1.0)

    def test_battery_sag_uses_measured_current_and_respects_thrust_cap(self):
        config = {
            "static_current_model": {"points": [
                {"throttle_percent": 0, "current_a": 0.0},
                {"throttle_percent": 50, "current_a": 10.0},
                {"throttle_percent": 100, "current_a": 30.0},
            ]},
            "battery_dynamics": {
                "enabled": True, "reference_voltage_v": 14.8,
                "full_voltage_v": 14.8, "empty_voltage_v": 13.2,
                "minimum_loaded_voltage_v": 12.0, "capacity_ah": 10.0,
                "pack_internal_resistance_ohm": 0.01,
                "thrust_voltage_exponent": 2.0,
            },
        }
        self.assertAlmostEqual(command_to_current_a(config, 0.25), 5.0)
        soc, voltage, scale, current = battery_step(
            config, np.full(8, 0.5), 1.0, 1.0
        )
        self.assertAlmostEqual(current, 80.0)
        self.assertLess(soc, 1.0)
        self.assertLess(voltage, 14.8)
        self.assertGreater(scale, 0.0)
        self.assertLess(scale, 1.0)
        _, _, disabled_scale, disabled_current = battery_step(
            {}, np.ones(8), 1.0, 1.0
        )
        self.assertEqual(disabled_scale, 1.0)
        self.assertEqual(disabled_current, 0.0)

    def test_actuator_transport_delay_uses_simulation_time(self):
        pending = deque([
            (1.000, np.full(8, 0.2)),
            (1.004, np.full(8, 0.6)),
        ])
        current = np.zeros(8)
        current = delayed_command_step(pending, current, 1.004, 0.008)
        np.testing.assert_allclose(current, np.zeros(8))
        current = delayed_command_step(pending, current, 1.008, 0.008)
        np.testing.assert_allclose(current, np.full(8, 0.2))
        current = delayed_command_step(pending, current, 1.012, 0.008)
        np.testing.assert_allclose(current, np.full(8, 0.6))
        self.assertFalse(pending)

    def test_gazebo_sensor_delay_uses_message_simulation_stamp(self):
        class Stamp:
            sec = 12
            nsec = 345000000
        class Header:
            stamp = Stamp()
        class Message:
            header = Header()
        self.assertAlmostEqual(message_stamp_seconds(Message()), 12.345)
        self.assertAlmostEqual(message_stamp_seconds(object(), 7.5), 7.5)

    def test_reachable_inverse_kinematics(self):
        source = {
            "shoulder_pan": 0.4,
            "shoulder_lift": -0.6,
            "elbow_flex": 0.8,
            "wrist_flex": -0.5,
            "wrist_roll": 0.3,
        }
        target, _ = self.model.forward_kinematics(
            "gripper_frame_link", source
        )
        solution, status = solve_ik(self.model, target)
        achieved, _ = self.model.forward_kinematics(
            "gripper_frame_link", solution
        )
        self.assertTrue(status["converged"])
        self.assertLess(
            float(np.linalg.norm(target[:3, 3] - achieved[:3, 3])),
            1e-4,
        )

    def test_floating_base_momentum_conservation(self):
        positions = {
            "shoulder_pan": 0.4,
            "shoulder_lift": -0.6,
            "elbow_flex": 0.8,
            "wrist_flex": -0.5,
            "wrist_roll": 0.3,
        }
        velocities = {name: value for name, value in positions.items()}
        base_twist, relative_momentum, residual = reaction_twist(
            self.model, positions, velocities
        )
        self.assertGreater(float(np.linalg.norm(relative_momentum)), 1e-4)
        self.assertGreater(float(np.linalg.norm(base_twist)), 1e-4)
        self.assertLess(float(np.linalg.norm(residual)), 1e-9)

    def test_allocation_and_both_flight_modes(self):
        matrix = allocation_matrix(self.config)
        self.assertEqual(int(np.linalg.matrix_rank(matrix)), 6)
        for mode in ("direct", "motors"):
            summary, _ = simulate(
                self.model,
                self.config,
                mode=mode,
                duration=8.0,
                dt=0.002,
            )
            self.assertLess(
                float(np.linalg.norm(summary["position_error"])), 0.001
            )
            self.assertLess(
                float(np.linalg.norm(summary["attitude_error"])), 0.001
            )

    def test_gazebo_wrench_equilibrium_and_com_offset(self):
        mass, center, inertia = self.model.mass_properties({})
        force, torque_at_origin = compute_wrench_enu(
            mass,
            inertia,
            center,
            np.array([0.0, 0.0, 1.0]),
            np.zeros(3),
            np.eye(3),
            np.zeros(3),
            np.array([0.0, 0.0, 1.0]),
        )
        np.testing.assert_allclose(force, [0.0, 0.0, mass * 9.81])
        moment_at_com = torque_at_origin - np.cross(center, force)
        np.testing.assert_allclose(moment_at_com, np.zeros(3), atol=1e-12)

    def test_legacy_virtual_rotor_artifact_is_well_formed(self):
        robot = ET.parse(OCTOROTOR_URDF).getroot()
        link_names = {link.attrib["name"] for link in robot.findall("link")}
        self.assertIn("base_link", link_names)
        self.assertIn("arm_base_link", link_names)
        self.assertNotIn("drone_base_link", link_names)
        sensors = robot.findall("./gazebo[@reference='base_link']/sensor")
        self.assertEqual(
            {sensor.attrib["name"] for sensor in sensors},
            {
                "air_pressure_sensor",
                "magnetometer_sensor",
                "imu_sensor",
                "navsat_sensor",
            },
        )
        plugins = [
            plugin
            for plugin in robot.findall("./gazebo/plugin")
            if plugin.attrib.get("filename")
            == "gz-sim-multicopter-motor-model-system"
        ]
        self.assertEqual(len(plugins), 8)
        self.assertTrue(
            all(
                plugin.findtext("commandSubTopic")
                == "command/motor_speed"
                for plugin in plugins
            )
        )
        for rotor in self.config["rotors"]:
            self.assertIsNotNone(
                robot.find(f"./joint[@name='{rotor['name']}_joint']")
            )

    def test_legacy_measured_geometry_mass_profiles_match(self):
        config = json.loads(MEASURED_CONFIG_PATH.read_text(encoding="utf-8"))
        extracted = json.loads(EXTRACTED_MOUNTS_PATH.read_text(encoding="utf-8"))
        by_motor = {int(rotor["motor"]): rotor for rotor in config["rotors"]}
        self.assertEqual(set(by_motor), set(range(1, 9)))
        for mount in extracted["mounts"]:
            rotor = by_motor[int(mount["motor"])]
            np.testing.assert_allclose(
                rotor["position_m"], mount["position_frd_m"], atol=1e-9
            )
            np.testing.assert_allclose(
                rotor["axis_body"], mount["axis_frd"], atol=1e-9
            )

        matrix = allocation_matrix(config)
        self.assertEqual(int(np.linalg.matrix_rank(matrix)), 6)
        maximum = float(config["maximum_thrust_n"])
        vertical_force = maximum * sum(
            -np.asarray(rotor["axis_body"], dtype=float)[2]
            for rotor in config["rotors"]
        )
        self.assertAlmostEqual(
            vertical_force, config["maximum_vertical_force_n"], places=7
        )
        for profile, should_hover in (
            ("flight_test", True),
            ("physical_estimate", False),
        ):
            mass = config["mass_profiles"][profile]["all_up_mass_kg"]
            desired = np.array([0.0, 0.0, -mass * 9.80665, 0.0, 0.0, 0.0])
            thrust = np.linalg.pinv(matrix) @ desired
            self.assertGreater(float(np.min(thrust)), 0.0)
            self.assertEqual(float(np.max(thrust)) <= maximum, should_hover)

        for path, profile in (
            (MEASURED_FLIGHT_URDF, "flight_test"),
            (MEASURED_PHYSICAL_URDF, "physical_estimate"),
        ):
            model = UrdfModel(path)
            mass, _, inertia = model.mass_properties({})
            self.assertAlmostEqual(
                mass, config["mass_profiles"][profile]["all_up_mass_kg"], places=7
            )
            self.assertGreater(float(np.min(np.linalg.eigvalsh(inertia))), 0.0)

        robot = ET.parse(MEASURED_FLIGHT_URDF).getroot()
        sensors = robot.findall(
            "./gazebo[@reference='drone_base_link']/sensor"
        )
        self.assertEqual(
            {sensor.attrib["name"] for sensor in sensors},
            {
                "air_pressure_sensor",
                "magnetometer_sensor",
                "imu_sensor",
                "navsat_sensor",
            },
        )
        for rotor in config["rotors"]:
            joint = robot.find(
                f"./joint[@name='{rotor['name']}_joint']"
            )
            rpy = np.asarray(
                [float(value) for value in joint.find("origin").attrib["rpy"].split()]
            )
            axis_frd = np.asarray(rotor["axis_body"], dtype=float)
            expected_flu = np.array(
                [axis_frd[0], -axis_frd[1], -axis_frd[2]]
            )
            expected_flu /= np.linalg.norm(expected_flu)
            position_frd = np.asarray(rotor["position_m"], dtype=float)
            expected_position_flu = np.array(
                [position_frd[0], -position_frd[1], -position_frd[2]]
            )
            actual_position_flu = np.asarray(
                [
                    float(value)
                    for value in joint.find("origin").attrib["xyz"].split()
                ]
            )
            np.testing.assert_allclose(
                actual_position_flu,
                expected_position_flu,
                atol=1e-9,
            )
            np.testing.assert_allclose(
                _rpy_matrix(rpy) @ np.array([0.0, 0.0, 1.0]),
                expected_flu,
                atol=1e-7,
            )
        commands = np.ones(8)
        force_flu, torque_flu = direct_wrench_flu(config, commands)
        expected_frd = matrix @ np.full(8, maximum)
        np.testing.assert_allclose(force_flu, FRD_TO_FLU @ expected_frd[:3])
        np.testing.assert_allclose(torque_flu, FRD_TO_FLU @ expected_frd[3:])

    def test_cad_v2_is_split_and_dynamically_allocatable(self):
        config = json.loads(CAD_V2_CONFIG_PATH.read_text(encoding="utf-8"))
        manifest = json.loads(CAD_MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            manifest["assembly_sha256"], config["assembly_sha256"]
        )
        self.assertTrue(manifest["assembly"].endswith("组合无人机.SLDASM"))
        self.assertEqual(manifest["component_count"], 442)
        self.assertEqual(len(config["rotors"]), 8)

        robot = ET.parse(CAD_V2_URDF).getroot()
        self.assertEqual(len(robot.findall("link")), 24)
        self.assertEqual(len(robot.findall("joint[@type='continuous']")), 8)
        self.assertEqual(len(robot.findall("joint[@type='revolute']")), 6)
        model = UrdfModel(CAD_V2_URDF)
        mass, _, inertia = model.mass_properties({})
        self.assertAlmostEqual(
            mass, config["estimated_all_up_mass_kg"], places=7
        )
        self.assertGreater(float(np.min(np.linalg.eigvalsh(inertia))), 0.0)

        matrix = allocation_matrix(config)
        self.assertEqual(int(np.linalg.matrix_rank(matrix)), 6)
        upward = {
            int(rotor["motor"])
            for rotor in config["rotors"]
            if float(rotor["axis_body"][2]) < 0.0
        }
        self.assertEqual(upward, {1, 2, 3, 6})
        self.assertEqual(config["flight_feasibility_nonreversible"], "INFEASIBLE")
        self.assertLess(config["best_case_thrust_to_weight_nonreversible"], 1.0)

        airframe = AIRFRAME_PATH.read_text(encoding="utf-8")
        for rotor in config["rotors"]:
            transform, _ = model.forward_kinematics(rotor["name"], {})
            actual_axis_flu = transform[:3, :3] @ np.array([0.0, 0.0, 1.0])
            expected_axis_frd = np.asarray(rotor["axis_body"], dtype=float)
            np.testing.assert_allclose(
                actual_axis_flu,
                FRD_TO_FLU @ expected_axis_frd,
                atol=1e-7,
            )
            px4_index = int(rotor["motor"]) - 1
            expected_values = {
                "PX": rotor["position_m"][0],
                "PY": rotor["position_m"][1],
                "PZ": rotor["position_m"][2],
                "AX": rotor["axis_body"][0],
                "AY": rotor["axis_body"][1],
                "AZ": rotor["axis_body"][2],
                "KM": 0.0,
            }
            for suffix, expected in expected_values.items():
                match = re.search(
                    rf"CA_ROTOR{px4_index}_{suffix}\s+([-+0-9.eE]+)",
                    airframe,
                )
                self.assertIsNotNone(match)
                self.assertAlmostEqual(float(match.group(1)), expected, places=6)
        for motor in range(1, 9):
            self.assertRegex(airframe, rf"SIM_GZ_EC_MAX{motor}\s+1000\b")

    def test_formal_cad_urdf_closes_aggregate_mass_com_and_inertia(self):
        report = json.loads(CAD_V3_FORMAL_REPORT.read_text(encoding="utf-8"))
        physical = json.loads(
            (WORKSPACE / "analysis" / "cad_direct" / "cad_mass_properties.json")
            .read_text(encoding="utf-8")
        )
        active_physical = json.loads(CAD_V3_CONFIG_PATH.read_text(encoding="utf-8"))
        model = UrdfModel(CAD_V3_FORMAL_URDF)
        mass, center, inertia = model.mass_properties({})
        self.assertAlmostEqual(mass, active_physical["estimated_mass_kg"], places=7)
        self.assertAlmostEqual(
            active_physical["cad_density_estimated_mass_kg"],
            physical["estimated_total_mass_kg"],
            places=7,
        )
        np.testing.assert_allclose(center, report["formal_com_ros_flu_m"], atol=1e-8)
        np.testing.assert_allclose(
            inertia,
            np.asarray(active_physical["estimated_ros_flu_inertia_at_com_kg_m2"]),
            atol=1e-8,
        )
        self.assertGreater(float(np.min(np.linalg.eigvalsh(inertia))), 0.0)
        self.assertEqual(
            report["flight_status_at_1p2_kgf_per_motor"],
            "FEASIBLE_WITH_OPPOSITE_PITCH_HYPOTHESIS",
        )

    def test_7p735_flight_config_preserves_cad_evidence_and_hover_margin(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        physical = json.loads(CAD_V3_CONFIG_PATH.read_text(encoding="utf-8"))
        source = json.loads(
            (WORKSPACE / "analysis" / "cad_direct" / "cad_mass_properties.json")
            .read_text(encoding="utf-8")
        )
        self.assertAlmostEqual(config["estimated_all_up_mass_kg"], 7.735)
        self.assertAlmostEqual(
            physical["cad_density_estimated_mass_kg"],
            source["estimated_total_mass_kg"],
        )
        self.assertAlmostEqual(
            physical["inertia_scaling_from_cad_density_estimate"],
            7.735 / source["estimated_total_mass_kg"],
        )
        self.assertEqual(
            config["flight_feasibility_nonreversible"],
            "FEASIBLE_WITH_OPPOSITE_PITCH_HYPOTHESIS",
        )
        self.assertLessEqual(
            max(config["bounded_hover_thrust_n"]), config["maximum_thrust_n"]
        )
        self.assertLess(config["bounded_hover_residual_norm"], 1e-10)
        self.assertFalse(config["battery_dynamics"]["enabled"])

    def test_7p735_px4_hover_uses_static_thrust_inverse(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        mean_hover_thrust = float(np.mean(config["bounded_hover_thrust_n"]))
        hover_command = _thrust_to_command(config, mean_hover_thrust)
        self.assertAlmostEqual(hover_command, 0.873727, places=6)
        self.assertNotAlmostEqual(
            hover_command,
            mean_hover_thrust / config["maximum_thrust_n"],
            places=2,
        )
        airframe = CAD_V3_AIRFRAME_PATH.read_text(encoding="utf-8")
        match = re.search(r"MPC_THR_HOVER\s+([-+0-9.eE]+)", airframe)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(float(match.group(1)), hover_command, places=4)

    def test_7p735_allocation_and_wrench_reference_points_are_distinct(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        matrix = allocation_matrix(config)
        self.assertEqual(int(np.linalg.matrix_rank(matrix)), 6)
        self.assertTrue(
            any(
                not np.allclose(rotor["position_m"], rotor["wrench_position_m"])
                for rotor in config["rotors"]
            )
        )
        expected_frd = matrix @ np.asarray(config["bounded_hover_thrust_n"])
        np.testing.assert_allclose(
            expected_frd,
            [0.0, 0.0, -7.735 * 9.80665, 0.0, 0.0, 0.0],
            atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main()
