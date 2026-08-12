"""Regression tests for the my_drone simulation artifacts."""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path
import re
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix, allocate_bounded_wrench
from drone_arm_sim.flight_control_demo import simulate
from drone_arm_sim.floating_base_reaction import reaction_twist
from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES, Payload
from drone_arm_sim.arm_coupling_monitor import (
    bounded_compensation_ned,
    filtered_joint_acceleration_step,
)
from drone_arm_sim.arm_disturbance_observer import (
    BoundedTorqueDisturbanceObserver,
    inertia_tensor_flu_to_frd,
    motor_torque_about_dynamic_com_frd,
    parse_cli_and_ros_args,
    rigid_body_residual_torque_frd,
)
from drone_arm_sim.gazebo_wrench_controller import compute_wrench_enu
from drone_arm_sim.gazebo_direct_motor_model import (
    FRD_TO_FLU,
    arm_compensation_torque_step,
    battery_step,
    command_to_current_a,
    command_to_thrust_n,
    actuator_command_to_thrust_n,
    thrust_to_actuator_command,
    direct_wrench_flu,
    delayed_command_step,
    environment_wrench_world,
    first_order_motor_step,
    ground_effect_thrust_scale,
    motor_wrench_frd,
    per_motor_wrench_frd,
    landing_support_restore_ready,
    takeoff_support_release_ready,
    thrust_to_command,
    torque_feedforward_command_delta,
)
from drone_arm_sim.inverse_kinematics import solve_ik
from drone_arm_sim.cartesian_arm_demo import (
    TOOL_FORWARD_AXIS_LOCAL,
    _trajectory_points,
    plan_tool_forward_path,
)
from drone_arm_sim.gazebo_sensor_delay import message_stamp_seconds
from drone_arm_sim.model_analysis import UrdfModel, _rpy_matrix
from drone_arm_sim.motor_evidence import validate_motor_thrust_evidence
from drone_arm_sim.rl_env import make_default_env


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
# The airframe generator is a source-side helper, not an installed ROS module.
# Add its directory explicitly so tests work from a clean colcon environment
# (and do not accidentally import an unrelated ``scripts`` package).
sys.path.insert(0, str(PACKAGE / "scripts"))
from generate_cad_px4_airframe import _thrust_to_command
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
CAD_V3_DEBUG_4KG_CONFIG_PATH = (
    PACKAGE / "config" / "my_drone_v3_cad_debug_4kg.json"
)
CAD_V3_AIRFRAME_PATH = (
    WORKSPACE / "px4" / "airframes" / "4026_gz_my_drone_octorotor_7p735"
)
SO101_MOTION_REFERENCE_PATH = PACKAGE / "config" / "so101_motion_reference.json"
GRASP_WORLD_PATH = PACKAGE / "worlds" / "flight_world_grasp_test.sdf"
FLIGHT_WORLD_PATH = PACKAGE / "worlds" / "flight_world_250hz.sdf"
LANDING_SUPPORT_PATH = PACKAGE / "worlds" / "landing_support.sdf"
LATEST_WRENCH_SOURCE = (
    PACKAGE.parent / "drone_motor_system" / "src" / "LatestWrenchSystem.cc"
)
CAD_DIRECT_LAUNCH = PACKAGE / "launch" / "cad_direct_thrust.launch.py"
CAD_MANIFEST_PATH = WORKSPACE / "analysis" / "cad_direct" / "assembly_manifest.json"
MOTOR_FREEZE_STATUS_PATH = (
    WORKSPACE / "analysis" / "cad_direct" / "motor_physical_freeze_status.json"
)
MOTOR_AXIS_EVIDENCE_PATH = (
    WORKSPACE / "analysis" / "cad_direct" / "motor_axis_evidence.json"
)
MOTOR_EVIDENCE_TEMPLATE_PATH = (
    WORKSPACE / "analysis" / "cad_direct" / "motor_thrust_evidence.template.json"
)
PROPELLER_PITCH_EVIDENCE_PATH = (
    WORKSPACE / "analysis" / "cad_direct" / "propeller_pitch_geometry_evidence.json"
)
PROPELLER_METADATA_EVIDENCE_PATH = (
    WORKSPACE / "analysis" / "cad_direct" / "propeller_metadata_evidence.json"
)
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
        self.assertAlmostEqual(thrust_to_command(config, 5.34462425), 0.50)
        self.assertAlmostEqual(thrust_to_command(config, 11.76798), 0.90)

    def test_arm_torque_feedforward_uses_formal_allocation_and_is_bounded(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        commands = np.full(8, 0.874)
        reaction_frd = np.array([0.10, -0.08, 0.05])
        compensated, delta = torque_feedforward_command_delta(
            config, commands, reaction_frd, max_delta_n=2.0
        )
        self.assertEqual(compensated.shape, (8,))
        self.assertTrue(np.all((compensated >= 0.0) & (compensated <= 1.0)))
        # A trim is deliberately small in normalized command space and must
        # not replace the PX4 hover command.
        self.assertLess(float(np.max(np.abs(delta))), 0.25)
        base_force, base_torque = direct_wrench_flu(config, commands)
        trim_force, trim_torque = direct_wrench_flu(config, compensated)
        self.assertGreater(float(np.linalg.norm(trim_torque - base_torque)), 1e-4)
        self.assertLess(float(np.linalg.norm(trim_force - base_force)), 1.0)

    def test_4kg_observer_trim_realizes_opposite_torque_without_net_force(self):
        """Check the observer sign through the actual 4 kg 6x8 allocator.

        This is deliberately stronger than checking that motor commands merely
        changed: away from actuator limits, the resulting rotor-wrench delta
        must be the requested negative disturbance torque and approximately
        zero force in PX4 FRD.
        """
        config = json.loads(
            CAD_V3_DEBUG_4KG_CONFIG_PATH.read_text(encoding="utf-8")
        )
        hover_commands = np.asarray(
            [
                thrust_to_actuator_command(config, thrust_n)
                for thrust_n in config["bounded_hover_thrust_n"]
            ],
            dtype=float,
        )
        disturbance_frd_nm = np.array([0.04, -0.03, 0.02])
        compensated, _ = torque_feedforward_command_delta(
            config,
            hover_commands,
            disturbance_frd_nm,
            max_delta_n=1.0,
        )
        base_force_flu, base_torque_flu = direct_wrench_flu(
            config, hover_commands
        )
        trim_force_flu, trim_torque_flu = direct_wrench_flu(
            config, compensated
        )
        force_delta_frd = FRD_TO_FLU @ (trim_force_flu - base_force_flu)
        torque_delta_frd = FRD_TO_FLU @ (trim_torque_flu - base_torque_flu)

        np.testing.assert_allclose(force_delta_frd, np.zeros(3), atol=1.0e-8)
        np.testing.assert_allclose(
            torque_delta_frd, -disturbance_frd_nm, atol=1.0e-8
        )
        self.assertTrue(np.all((compensated > 0.0) & (compensated < 1.0)))

    def test_static_com_low_pass_is_bounded_and_default_can_be_noop(self):
        from drone_arm_sim.gazebo_direct_motor_model import first_order_vector_step

        target = np.array([0.0, 0.624, 0.0])
        unchanged = first_order_vector_step(np.zeros(3), target, 0.0, 5.0)
        np.testing.assert_allclose(unchanged, np.zeros(3))
        first = first_order_vector_step(np.zeros(3), target, 0.1, 5.0)
        self.assertGreater(first[1], 0.0)
        self.assertLess(first[1], target[1])
        instant = first_order_vector_step(np.zeros(3), target, 0.1, 0.0)
        np.testing.assert_allclose(instant, target)

    def test_static_com_compensation_persists_after_arm_motion_stops(self):
        combined, filtered, active = arm_compensation_torque_step(
            np.zeros(3),
            np.array([0.2, 0.0, 0.0]),
            np.array([0.0, 0.6, 0.0]),
            reaction_available=False,
            gravity_available=True,
            gravity_gain=0.5,
            dt_s=0.1,
            gravity_time_constant_s=0.0,
        )
        self.assertTrue(active)
        np.testing.assert_allclose(filtered, [0.0, 0.6, 0.0])
        # The transient reaction is absent, but the pose-dependent gravity
        # moment remains and therefore still produces an allocator request.
        np.testing.assert_allclose(combined, [0.0, 0.3, 0.0])

    def test_arm_reaction_and_static_com_have_independent_gates(self):
        combined, filtered, active = arm_compensation_torque_step(
            np.zeros(3),
            np.array([0.2, -0.1, 0.05]),
            np.array([0.0, 0.6, 0.0]),
            reaction_available=True,
            gravity_available=False,
            gravity_gain=1.0,
            dt_s=0.1,
            gravity_time_constant_s=0.0,
        )
        self.assertTrue(active)
        np.testing.assert_allclose(filtered, np.zeros(3))
        np.testing.assert_allclose(combined, [0.2, -0.1, 0.05])

    def test_cad_flight_reaction_torque_signs_match_cw_ccw(self):
        config = json.loads(
            (PACKAGE / "config" / "my_drone_v2_cad_flight_pitch_corrected.json")
            .read_text(encoding="utf-8")
        )
        self.assertAlmostEqual(config["reaction_moment_ratio_m"], 0.001)
        self.assertEqual(
            config["reaction_moment_estimate"]["status"],
            "initial formal estimate; not a measured propeller parameter",
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

    def test_propeller_cad_does_not_claim_unencoded_opposite_pitch(self):
        evidence = json.loads(
            PROPELLER_PITCH_EVIDENCE_PATH.read_text(encoding="utf-8")
        )
        reuse = evidence["component_reuse_evidence"]
        self.assertEqual(evidence["result"], "PITCH_GEOMETRY_UNRESOLVED")
        self.assertEqual(reuse["unique_source_part_count"], 1)
        self.assertTrue(reuse["all_instance_transform_determinants_positive"])
        self.assertEqual(reuse["mirrored_motor_instances"], [])
        self.assertLess(evidence["maximum_p95_abs_pitch_angle_deg"], 0.01)

    def test_propeller_metadata_contains_no_hidden_handedness_designation(self):
        metadata = json.loads(
            PROPELLER_METADATA_EVIDENCE_PATH.read_text(encoding="utf-8")
        )
        pitch = json.loads(
            PROPELLER_PITCH_EVIDENCE_PATH.read_text(encoding="utf-8")
        )
        self.assertEqual(metadata["conclusion"], "NO_HANDEDNESS_METADATA_FOUND")
        self.assertEqual(metadata["handedness_tokens_found"], [])
        self.assertEqual(metadata["mirror_features"], [])
        self.assertIn(
            metadata["source_part_sha256"],
            pitch["component_reuse_evidence"]["unique_source_part_sha256"],
        )

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

    def test_7p735_px4_hover_uses_linear_scaled_thrust_boundary(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        mean_hover_thrust = float(np.mean(config["bounded_hover_thrust_n"]))
        hover_command = config["actuator_normalization"]["px4_hover_command"]
        self.assertEqual(
            config["actuator_input_model"],
            "hover_scaled_linear_thrust_with_rated_cap",
        )
        self.assertAlmostEqual(hover_command, 0.85, places=6)
        self.assertAlmostEqual(
            actuator_command_to_thrust_n(config, hover_command), mean_hover_thrust
        )
        self.assertAlmostEqual(
            thrust_to_actuator_command(config, mean_hover_thrust), hover_command
        )
        self.assertAlmostEqual(actuator_command_to_thrust_n(config, 0.0), 0.0)
        self.assertAlmostEqual(
            actuator_command_to_thrust_n(config, 1.0), config["maximum_thrust_n"]
        )
        rated_command = config["actuator_normalization"]["rated_thrust_command"]
        self.assertGreater(rated_command, hover_command)
        self.assertLess(rated_command, 1.0)
        self.assertAlmostEqual(
            thrust_to_actuator_command(config, config["maximum_thrust_n"]),
            rated_command,
        )
        hover_commands = np.asarray([
            thrust_to_actuator_command(config, value)
            for value in config["bounded_hover_thrust_n"]
        ])
        reconstructed = np.asarray([
            actuator_command_to_thrust_n(config, value) for value in hover_commands
        ])
        np.testing.assert_allclose(
            allocation_matrix(config) @ reconstructed,
            [0.0, 0.0, -7.735 * 9.80665, 0.0, 0.0, 0.0],
            atol=1e-9,
        )
        airframe = CAD_V3_AIRFRAME_PATH.read_text(encoding="utf-8")
        match = re.search(r"MPC_THR_HOVER\s+([-+0-9.eE]+)", airframe)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(float(match.group(1)), hover_command, places=4)
        self.assertRegex(airframe, r"HTE_THR_RANGE\s+0\.01")
        self.assertRegex(airframe, r"HTE_HT_ERR_INIT\s+0\.00")
        self.assertRegex(airframe, r"MPC_TKO_RAMP_T\s+0\.80")
        self.assertRegex(airframe, r"MPC_Z_VEL_I_ACC\s+0\.35")

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

    def test_7p735_origin_wrench_transforms_exactly_to_com_allocation(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        com_offsets = np.asarray([
            np.asarray(rotor["wrench_position_m"], dtype=float)
            - np.asarray(rotor["position_m"], dtype=float)
            for rotor in config["rotors"]
        ])
        np.testing.assert_allclose(
            com_offsets, np.tile(com_offsets[0], (len(com_offsets), 1)), atol=2e-5
        )
        com_frd = np.mean(com_offsets, axis=0)
        origin_matrix = allocation_matrix(config, position_key="wrench_position_m")
        transformed = origin_matrix.copy()
        for column in range(transformed.shape[1]):
            transformed[3:, column] -= np.cross(com_frd, transformed[:3, column])
        np.testing.assert_allclose(transformed, allocation_matrix(config), atol=2e-5)

    def test_7p735_motor_dynamics_table_is_explicit_and_rpm_honest(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        table = config["motor_dynamics_table"]
        self.assertEqual(len(table["motors"]), 8)
        self.assertEqual(table["input"], "PX4 normalized_thrust in [0,1]; no RPM telemetry is available")
        self.assertIsNone(table["motors"][0]["thrust_coefficient_kf"])
        self.assertIsNone(table["motors"][0]["reaction_torque_coefficient_kq"])
        for item in table["motors"]:
            self.assertEqual(item["px4_output"], item["motor"] - 1)
            self.assertAlmostEqual(item["max_thrust_n"], 11.76798)
            self.assertAlmostEqual(item["rise_time_constant_s"], 0.035)
            self.assertAlmostEqual(item["fall_time_constant_s"], 0.035)

    def test_7p735_ground_contact_pads_release_for_takeoff(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        release = config["takeoff_support_release"]
        self.assertTrue(release["enabled"])
        self.assertEqual(
            release["model_name"], "my_drone_bringup_landing_support"
        )
        self.assertGreaterEqual(release["release_up_force_n"], 0.95 * 7.735 * 9.80665)
        hover_vertical_force = -allocation_matrix(config)[:3] @ np.asarray(
            config["bounded_hover_thrust_n"]
        )
        self.assertLess(release["release_up_force_n"], hover_vertical_force[2])
        self.assertLessEqual(release["maximum_horizontal_force_n"], 0.50)
        self.assertLessEqual(release["maximum_com_torque_nm"], 0.05)
        self.assertGreaterEqual(release["hold_time_s"], 0.15)
        self.assertTrue(release["restore_on_land"])
        self.assertGreaterEqual(release["restore_clearance_m"], 0.03)
        self.assertLessEqual(release["restore_clearance_m"], 0.10)
        restore_path = (CAD_V3_FLIGHT_CONFIG_PATH.parent / release["restore_sdf_filename"]).resolve()
        self.assertEqual(restore_path, LANDING_SUPPORT_PATH.resolve())
        self.assertTrue(restore_path.is_file())

        hover_wrench = allocation_matrix(config) @ np.asarray(
            config["bounded_hover_thrust_n"]
        )
        self.assertTrue(takeoff_support_release_ready(
            hover_wrench,
            release["release_up_force_n"],
            release["maximum_horizontal_force_n"],
            release["maximum_com_torque_nm"],
        ))
        excessive_lateral = hover_wrench.copy()
        excessive_lateral[0] = release["maximum_horizontal_force_n"] + 0.01
        self.assertFalse(takeoff_support_release_ready(
            excessive_lateral,
            release["release_up_force_n"],
            release["maximum_horizontal_force_n"],
            release["maximum_com_torque_nm"],
        ))
        excessive_torque = hover_wrench.copy()
        excessive_torque[4] = release["maximum_com_torque_nm"] + 0.01
        self.assertFalse(takeoff_support_release_ready(
            excessive_torque,
            release["release_up_force_n"],
            release["maximum_horizontal_force_n"],
            release["maximum_com_torque_nm"],
        ))

        clearance = release["restore_clearance_m"]
        self.assertFalse(landing_support_restore_ready(1.5, 0.8, clearance))
        self.assertTrue(landing_support_restore_ready(0.84, 0.8, clearance))
        self.assertFalse(landing_support_restore_ready(0.84, None, clearance))

    def test_each_motor_command_produces_force_and_torque(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        commands = np.array([0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85])
        records = per_motor_wrench_frd(config, commands)
        self.assertEqual(len(records), 8)
        thrust = np.array([record["thrust_n"] for record in records])
        wrench = np.concatenate((
            sum((record["force_frd_n"] for record in records), start=np.zeros(3)),
            sum((record["torque_frd_nm"] for record in records), start=np.zeros(3)),
        ))
        np.testing.assert_allclose(
            wrench,
            allocation_matrix(config, position_key="wrench_position_m") @ thrust,
            atol=1e-10,
        )
        for motor in range(1, 9):
            one_thrust, one_force, one_torque = motor_wrench_frd(config, motor, commands[motor - 1])
            np.testing.assert_allclose(one_force, records[motor - 1]["force_frd_n"])
            np.testing.assert_allclose(one_torque, records[motor - 1]["torque_frd_nm"])
            self.assertGreaterEqual(one_thrust, 0.0)

    def test_bounded_allocation_reports_hover_and_saturation(self):
        config = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        hover = np.array([0.0, 0.0, -7.735 * 9.80665, 0.0, 0.0, 0.0])
        result = allocate_bounded_wrench(config, hover)
        self.assertTrue(result["feasible"])
        self.assertLess(result["residual_norm"], 1e-8)
        self.assertEqual(result["thrust_n"].shape, (8,))
        impossible = allocate_bounded_wrench(
            config, np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        )
        self.assertTrue(np.any(impossible["saturated_high"]))
        self.assertGreater(impossible["residual_norm"], 1.0)

    def test_micro_flight_presets_are_limited_and_inside_joint_bounds(self):
        reference = json.loads(SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8"))
        joints = {item["name"]: item for item in reference["joints"]}
        retracted = reference["presets"]["retracted"]
        for name in ("flight_micro_a", "flight_micro_b"):
            target = reference["presets"][name]
            for index, joint_name in enumerate((
                "shoulder_pan", "shoulder_lift", "elbow_flex",
                "wrist_flex", "wrist_roll", "gripper",
            )):
                limit = joints[joint_name]
                self.assertGreaterEqual(target[index], limit["lower_rad"])
                self.assertLessEqual(target[index], limit["upper_rad"])
                self.assertLessEqual(abs(target[index] - retracted[index]), 0.04)

    def test_visible_demo_preset_is_bounded_and_clearly_extends_arm(self):
        reference = json.loads(SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8"))
        names = (
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper",
        )
        joints = {item["name"]: item for item in reference["joints"]}
        target = reference["presets"]["demo_extended"]
        for name, value in zip(names, target):
            self.assertGreaterEqual(value, joints[name]["lower_rad"])
            self.assertLessEqual(value, joints[name]["upper_rad"])

        model = UrdfModel(CAD_V3_FORMAL_URDF)
        home_map = model.link_transforms(dict(zip(names, reference["presets"]["retracted"])))
        demo_map = model.link_transforms(dict(zip(names, target)))
        shoulder = home_map["shoulder_link"][:3, 3]
        home_reach = float(np.linalg.norm(home_map["gripper_link"][:3, 3] - shoulder))
        demo_reach = float(np.linalg.norm(demo_map["gripper_link"][:3, 3] - shoulder))
        self.assertGreater(demo_reach - home_reach, 0.15)

    def test_cartesian_demo_is_straight_bounded_and_returns_exact_path(self):
        model = UrdfModel(CAD_V3_FORMAL_URDF)
        start = {
            name: 0.0 for name in (
                "shoulder_pan", "shoulder_lift", "elbow_flex",
                "wrist_flex", "wrist_roll", "gripper",
            )
        }
        path, evidence = plan_tool_forward_path(model, start)
        self.assertEqual(len(path), 25)
        origin = np.asarray(evidence["start_position_m"])
        axis = np.asarray(evidence["tool_forward_axis_base_at_start"])
        for index, positions in enumerate(path):
            transform, _ = model.forward_kinematics("gripper_link", positions)
            displacement = transform[:3, 3] - origin
            expected = 0.005 * index
            self.assertLess(float(np.linalg.norm(np.cross(displacement, axis))), 2.0e-4)
            self.assertAlmostEqual(float(displacement @ axis), expected, delta=2.0e-4)
            for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
                lower, upper = model.joint_limits(name)
                margin = np.deg2rad(5.0)
                self.assertGreaterEqual(positions[name], lower + margin)
                self.assertLessEqual(positions[name], upper - margin)
        self.assertTrue(np.allclose(TOOL_FORWARD_AXIS_LOCAL, [0.999961256, -0.000000665, 0.008802682]))
        points = _trajectory_points(path, 0.0, 8.0, 3.0)
        outward_count = len(path)
        returned = points[outward_count + 1:]
        self.assertEqual(len(returned), outward_count - 1)
        timestamps = [
            point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9
            for point in points
        ]
        self.assertTrue(all(right > left for left, right in zip(timestamps, timestamps[1:])))
        for expected, actual in zip(reversed(path[:-1]), returned):
            self.assertTrue(np.allclose(actual.positions[:5], [expected[name] for name in (
                "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll",
            )]))

    def test_formal_arm_position_hold_gain_is_nonoscillatory_and_not_weak(self):
        root = ET.parse(CAD_V3_FORMAL_URDF).getroot()
        gain = float(
            root.findtext(
                "./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
                "/position_proportional_gain"
            )
        )
        self.assertGreaterEqual(gain, 0.5)
        self.assertLessEqual(gain, 1.0)

    def test_coupled_arm_mass_properties_reaction_and_payload(self):
        reference = json.loads(SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8"))
        dynamics = CoupledArmDynamics(CAD_V3_FORMAL_URDF, reference, target_mass_kg=7.735)
        retracted = dict(zip(JOINT_NAMES, reference["presets"]["retracted"]))
        work = dict(zip(JOINT_NAMES, reference["presets"]["work_a"]))
        home = dynamics.state(retracted)
        moving = dynamics.state(
            work,
            velocities={name: 0.2 for name in JOINT_NAMES},
            accelerations={name: 0.3 for name in JOINT_NAMES},
        )
        loaded = dynamics.state(work, payload=Payload(0.25))
        self.assertAlmostEqual(home.mass_kg, 7.735, places=8)
        self.assertGreater(float(np.linalg.norm(moving.com_shift_m)), 1.0e-4)
        self.assertGreater(float(np.linalg.norm(moving.reaction_torque_body_nm)), 1.0e-4)
        self.assertGreater(loaded.mass_kg, home.mass_kg)
        self.assertGreater(float(np.linalg.norm(loaded.com_shift_m)), float(np.linalg.norm(moving.com_shift_m)) * 0.1)
        self.assertTrue(np.all(np.linalg.eigvalsh(loaded.inertia_at_com_kg_m2) > 0.0))
        impulse_twist = dynamics.contact_delta_twist(work, np.array([0.0, 0.0, -1.0]), Payload(0.25))
        self.assertGreater(float(np.linalg.norm(impulse_twist)), 1.0e-4)

    def test_formal_gripper_has_cad_collision_geometry_for_contact_bringup(self):
        root = ET.parse(CAD_V3_FORMAL_URDF).getroot()
        for link_name in ("gripper_link", "moving_jaw_link"):
            link = root.find(f"./link[@name='{link_name}']")
            self.assertIsNotNone(link)
            collisions = link.findall("collision")
            meshes = [c.find("./geometry/mesh") for c in collisions]
            meshes = [mesh for mesh in meshes if mesh is not None]
            self.assertTrue(meshes)
            self.assertTrue(any("so101_" in mesh.attrib["filename"] for mesh in meshes))
            self.assertTrue(
                any("support_collision" in c.get("name", "") for c in collisions)
            )

    def test_grasp_world_has_controlled_pose_and_contact_sensor(self):
        root = ET.parse(GRASP_WORLD_PATH).getroot()
        world = root.find("./world[@name='flight_world']")
        self.assertIsNotNone(world)
        object_model = world.find("./model[@name='grasp_test_object']")
        self.assertIsNotNone(object_model)
        self.assertNotIn("pose", object_model.attrib)
        pose = object_model.find("./pose")
        self.assertIsNotNone(pose)
        np.testing.assert_allclose(
            [float(value) for value in pose.text.split()[:3]],
            [0.12704, -0.06047, 0.6750],
            atol=1e-8,
        )
        sensor = object_model.find(
            "./link[@name='link']/sensor[@name='contact_sensor']"
        )
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib.get("type"), "contact")
        self.assertEqual(sensor.findtext("./contact/collision"), "collision")
        self.assertIsNotNone(world.find("./model[@name='grasp_test_pedestal']"))
        self.assertIsNotNone(world.find("./model[@name='aircraft_contact_fixture']"))

    def test_flight_world_has_marked_landing_support_for_retracted_cad_arm(self):
        root = ET.parse(FLIGHT_WORLD_PATH).getroot()
        world = root.find("./world[@name='flight_world']")
        self.assertIsNotNone(world)
        fixture = world.find(
            "./model[@name='my_drone_bringup_landing_support']"
        )
        self.assertIsNotNone(fixture)
        catch = world.find(
            "./model[@name='ground_plane']/link/collision[@name='emergency_catch_collision']"
        )
        self.assertIsNotNone(catch)
        self.assertLess(float(catch.findtext("./pose").split()[2]), -1.0)
        self.assertEqual(fixture.findtext("./static"), "true")
        supports = fixture.findall("./link")
        self.assertEqual(len(supports), 4)
        for link in supports:
            self.assertIsNotNone(link.find("./collision/geometry/box"))
            pose = [float(value) for value in link.findtext("./pose").split()]
            height = float(link.findtext("./collision/geometry/box/size").split()[2])
            self.assertAlmostEqual(pose[2] + height / 2.0, 0.0, places=6)
            friction = link.findtext("./collision/surface/friction/ode/mu")
            self.assertIsNotNone(friction)
            self.assertLessEqual(float(friction), 0.05)

        standalone = ET.parse(LANDING_SUPPORT_PATH).getroot().find(
            "./model[@name='my_drone_bringup_landing_support']"
        )
        self.assertIsNotNone(standalone)
        self.assertEqual(standalone.findtext("./static"), "true")
        landing_links = standalone.findall("./link")
        self.assertEqual(len(landing_links), 4)
        for link in landing_links:
            pose = [float(value) for value in link.findtext("./pose").split()]
            height = float(link.findtext("./collision/geometry/box/size").split()[2])
            # The landing fixture is deliberately 20 mm below the startup
            # support so AUTO LAND still observes downward motion at contact.
            self.assertAlmostEqual(pose[2] + height / 2.0, 0.797, places=6)

    def test_motor_wrench_is_held_by_physics_step_plugin(self):
        root = ET.parse(FLIGHT_WORLD_PATH).getroot()
        world = root.find("./world[@name='flight_world']")
        plugin = world.find(
            "./plugin[@name='drone_motor_system::LatestWrenchSystem']"
        )
        self.assertIsNotNone(plugin)
        self.assertEqual(
            plugin.attrib.get("filename"), "libdrone_latest_wrench_system.so"
        )
        self.assertEqual(
            plugin.findtext("./topic"), "/world/flight_world/wrench/latest"
        )
        launch_text = CAD_DIRECT_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("/world/flight_world/wrench/latest", launch_text)
        source = LATEST_WRENCH_SOURCE.read_text(encoding="utf-8")
        self.assertIn("ISystemPreUpdate", source)
        self.assertIn("link.AddWorldWrench", source)

    def test_isolated_gripper_presets_leave_other_joints_retracted(self):
        reference = json.loads(
            SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8")
        )
        retracted = np.asarray(reference["presets"]["retracted"], dtype=float)
        opened = np.asarray(reference["presets"]["gripper_open"], dtype=float)
        closed = np.asarray(reference["presets"]["gripper_closed"], dtype=float)
        np.testing.assert_allclose(opened[:5], retracted[:5], atol=1.0e-12)
        np.testing.assert_allclose(closed, retracted, atol=1.0e-12)
        self.assertGreater(opened[5], closed[5])

    def test_isolated_wrist_roll_presets_move_only_wrist_roll(self):
        reference = json.loads(
            SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8")
        )
        retracted = np.asarray(reference["presets"]["retracted"], dtype=float)
        moved = np.asarray(reference["presets"]["wrist_roll_test"], dtype=float)
        home = np.asarray(reference["presets"]["wrist_roll_home"], dtype=float)
        np.testing.assert_allclose(moved[[0, 1, 2, 3, 5]], retracted[[0, 1, 2, 3, 5]])
        np.testing.assert_allclose(home, retracted, atol=1.0e-12)
        self.assertNotAlmostEqual(moved[4], retracted[4], places=6)

    def test_isolated_shoulder_pan_presets_move_only_shoulder_pan(self):
        reference = json.loads(
            SO101_MOTION_REFERENCE_PATH.read_text(encoding="utf-8")
        )
        retracted = np.asarray(reference["presets"]["retracted"], dtype=float)
        moved = np.asarray(
            reference["presets"]["shoulder_pan_slow_test"], dtype=float
        )
        home = np.asarray(reference["presets"]["shoulder_pan_home"], dtype=float)
        np.testing.assert_allclose(moved[1:], retracted[1:], atol=1.0e-12)
        np.testing.assert_allclose(home, retracted, atol=1.0e-12)
        self.assertAlmostEqual(moved[0] - retracted[0], 0.10, places=9)

    def test_arm_feedforward_is_bounded_and_frame_converted(self):
        result = bounded_compensation_ned(
            np.array([0.0, 0.0, 7.735]), 7.735, np.eye(3), 0.6
        )
        np.testing.assert_allclose(result, [0.0, 0.0, 0.6])
        lateral = bounded_compensation_ned(
            np.array([7.735, 0.0, 0.0]), 7.735, np.eye(3), 0.6
        )
        np.testing.assert_allclose(lateral, [0.0, -0.6, 0.0])

    def test_joint_acceleration_filter_is_rate_independent_and_bounded(self):
        slow = filtered_joint_acceleration_step(0.0, 20.0, 0.01, 0.20, 4.0)
        expected = 20.0 * (1.0 - np.exp(-0.01 / 0.20))
        self.assertAlmostEqual(slow, expected)
        clipped = filtered_joint_acceleration_step(0.0, 200.0, 0.10, 0.20, 4.0)
        self.assertEqual(clipped, 4.0)
        unchanged = filtered_joint_acceleration_step(1.25, float("nan"), 0.01, 0.20, 4.0)
        self.assertEqual(unchanged, 1.25)

    def test_arm_dob_motor_model_matches_balanced_4kg_hover(self):
        config = json.loads(CAD_V3_DEBUG_4KG_CONFIG_PATH.read_text(encoding="utf-8"))
        hover_commands = np.array([
            thrust_to_actuator_command(config, thrust)
            for thrust in config["bounded_hover_thrust_n"]
        ])
        torque = motor_torque_about_dynamic_com_frd(
            config, hover_commands, np.zeros(3)
        )
        np.testing.assert_allclose(torque, np.zeros(3), atol=1.0e-8)

    def test_arm_dob_cli_preserves_ros_launch_arguments(self):
        parsed, ros_arguments = parse_cli_and_ros_args(
            [
                "--maximum-torque-nm",
                "0.07",
                "--ros-args",
                "-r",
                "__node:=arm_dob_test",
            ]
        )
        self.assertAlmostEqual(parsed.maximum_torque_nm, 0.07)
        self.assertEqual(
            ros_arguments,
            ["--ros-args", "-r", "__node:=arm_dob_test"],
        )

    def test_arm_dob_dynamic_com_shift_changes_rotor_moment_consistently(self):
        config = json.loads(CAD_V3_DEBUG_4KG_CONFIG_PATH.read_text(encoding="utf-8"))
        commands = np.full(8, 0.40)
        shift_flu = np.array([0.01, -0.02, 0.03])
        reference = motor_torque_about_dynamic_com_frd(config, commands)
        shifted = motor_torque_about_dynamic_com_frd(
            config, commands, shift_flu
        )
        records = per_motor_wrench_frd(config, commands)
        total_force_frd = sum(
            (record["force_frd_n"] for record in records), np.zeros(3)
        )
        shift_frd = FRD_TO_FLU @ shift_flu
        np.testing.assert_allclose(
            shifted - reference,
            -np.cross(shift_frd, total_force_frd),
            atol=1.0e-10,
        )

    def test_arm_dob_rigid_body_residual_obeys_euler_equation(self):
        inertia = np.array([0.2, 0.3, 0.4])
        omega = np.array([0.5, -0.2, 0.1])
        alpha = np.array([0.4, -0.1, 0.2])
        disturbance = np.array([0.03, -0.02, 0.01])
        motor = (
            inertia * alpha
            + np.cross(omega, inertia * omega)
            - disturbance
        )
        result = rigid_body_residual_torque_frd(alpha, omega, inertia, motor)
        np.testing.assert_allclose(result, disturbance, atol=1.0e-12)

    def test_arm_dob_rigid_body_residual_uses_full_inertia_tensor(self):
        inertia = np.array([
            [0.20, 0.012, -0.006],
            [0.012, 0.31, 0.009],
            [-0.006, 0.009, 0.43],
        ])
        omega = np.array([0.5, -0.2, 0.1])
        alpha = np.array([0.4, -0.1, 0.2])
        disturbance = np.array([0.03, -0.02, 0.01])
        motor = inertia @ alpha + np.cross(omega, inertia @ omega) - disturbance
        result = rigid_body_residual_torque_frd(alpha, omega, inertia, motor)
        np.testing.assert_allclose(result, disturbance, atol=1.0e-12)

    def test_arm_dob_inertia_tensor_flu_to_frd_flips_cross_term_signs(self):
        inertia_flu = np.array([
            [0.20, 0.012, -0.006],
            [0.012, 0.31, 0.009],
            [-0.006, 0.009, 0.43],
        ])
        converted = inertia_tensor_flu_to_frd(inertia_flu)
        expected = FRD_TO_FLU @ inertia_flu @ FRD_TO_FLU.T
        np.testing.assert_allclose(converted, expected, atol=1.0e-12)
        self.assertAlmostEqual(converted[0, 1], -inertia_flu[0, 1])
        self.assertAlmostEqual(converted[0, 2], -inertia_flu[0, 2])
        self.assertAlmostEqual(converted[1, 2], inertia_flu[1, 2])

    def test_arm_dob_is_baseline_gated_motion_only_and_bounded(self):
        observer = BoundedTorqueDisturbanceObserver(
            angular_acceleration_time_constant_s=0.0,
            baseline_time_constant_s=0.0,
            estimate_time_constant_s=0.0,
            decay_time_constant_s=0.0,
            maximum_torque_nm=0.08,
        )
        inertia = np.array([0.2, 0.21, 0.22])
        omega = np.zeros(3)
        for _ in range(25):
            torque, active = observer.step(
                angular_velocity_frd_rad_s=omega,
                predicted_motor_torque_frd_nm=np.zeros(3),
                inertia_diag_kg_m2=inertia,
                dt_s=0.01,
                armed=True,
                arm_motion_active=False,
            )
            self.assertFalse(active)
            np.testing.assert_array_equal(torque, np.zeros(3))

        omega = np.array([0.01, 0.0, 0.0])
        torque, active = observer.step(
            angular_velocity_frd_rad_s=omega,
            predicted_motor_torque_frd_nm=np.zeros(3),
            inertia_diag_kg_m2=inertia,
            dt_s=0.01,
            armed=True,
            arm_motion_active=True,
        )
        self.assertTrue(active)
        self.assertAlmostEqual(float(np.linalg.norm(torque)), 0.08)
        self.assertGreater(torque[0], 0.0)

        torque, active = observer.step(
            angular_velocity_frd_rad_s=omega,
            predicted_motor_torque_frd_nm=np.zeros(3),
            inertia_diag_kg_m2=inertia,
            dt_s=0.01,
            armed=True,
            arm_motion_active=False,
        )
        self.assertFalse(active)
        np.testing.assert_array_equal(torque, np.zeros(3))

    def test_arm_dob_subtracts_static_actuator_model_residual(self):
        observer = BoundedTorqueDisturbanceObserver(
            angular_acceleration_time_constant_s=0.0,
            baseline_time_constant_s=0.0,
            estimate_time_constant_s=0.0,
            maximum_torque_nm=0.08,
        )
        inertia = np.array([0.2, 0.21, 0.22])
        model_bias = np.array([0.04, -0.02, 0.01])
        for _ in range(25):
            observer.step(
                angular_velocity_frd_rad_s=np.zeros(3),
                predicted_motor_torque_frd_nm=model_bias,
                inertia_diag_kg_m2=inertia,
                dt_s=0.01,
                armed=True,
                arm_motion_active=False,
            )
        torque, active = observer.step(
            angular_velocity_frd_rad_s=np.zeros(3),
            predicted_motor_torque_frd_nm=model_bias,
            inertia_diag_kg_m2=inertia,
            dt_s=0.01,
            armed=True,
            arm_motion_active=True,
        )
        self.assertTrue(active)
        np.testing.assert_allclose(torque, np.zeros(3), atol=1.0e-12)

    def test_arm_dob_compensation_sign_reduces_constant_disturbance_error(self):
        def simulate(enabled: bool) -> np.ndarray:
            dt = 0.004
            inertia = np.diag([0.20, 0.21, 0.22])
            observer = BoundedTorqueDisturbanceObserver(maximum_torque_nm=0.08)
            angle = np.zeros(3)
            omega = np.zeros(3)
            estimate = np.zeros(3)
            # Establish the same armed, arm-static baseline required in flight.
            for _ in range(125):
                motor = -1.2 * angle - 0.30 * omega
                alpha = np.linalg.solve(inertia, motor)
                omega += alpha * dt
                angle += omega * dt
                estimate, _ = observer.step(
                    angular_velocity_frd_rad_s=omega,
                    predicted_motor_torque_frd_nm=motor,
                    inertia_diag_kg_m2=inertia,
                    dt_s=dt,
                    armed=True,
                    arm_motion_active=False,
                )
            history = []
            disturbance = np.array([0.04, 0.0, 0.0])
            for _ in range(750):
                # Production allocation requests the exact negative of the
                # observer estimate; retain the experimental 0.5 gain here.
                compensation = -0.5 * estimate if enabled else np.zeros(3)
                motor = -1.2 * angle - 0.30 * omega + compensation
                alpha = np.linalg.solve(inertia, motor + disturbance)
                omega += alpha * dt
                angle += omega * dt
                estimate, _ = observer.step(
                    angular_velocity_frd_rad_s=omega,
                    predicted_motor_torque_frd_nm=motor,
                    inertia_diag_kg_m2=inertia,
                    dt_s=dt,
                    armed=True,
                    arm_motion_active=True,
                )
                history.append(angle[0])
            return np.asarray(history)

        uncompensated = simulate(False)
        compensated = simulate(True)
        tail = slice(len(uncompensated) // 2, None)
        self.assertLess(
            float(np.sqrt(np.mean(compensated[tail] ** 2))),
            float(np.sqrt(np.mean(uncompensated[tail] ** 2))),
        )
        self.assertLess(abs(float(compensated[-1])), abs(float(uncompensated[-1])))

    def test_offline_rl_env_shapes_bounds_and_arm_coupling(self):
        env = make_default_env("arm_pose")
        reset = env.reset(seed=13)
        observation = reset[0] if isinstance(reset, tuple) else reset
        self.assertEqual(observation.shape, (25,))
        self.assertEqual(env.action_space.shape, (14,))
        np.testing.assert_allclose(env.action_space.low[:8], np.zeros(8))
        np.testing.assert_allclose(env.action_space.low[8:], -np.ones(6))
        action = env.hover_action()
        action[8:] = 0.4
        next_observation, reward, terminated, truncated, info = env.step(action)
        self.assertEqual(next_observation.shape, (25,))
        self.assertTrue(np.all(np.isfinite(next_observation)))
        self.assertTrue(np.isfinite(reward))
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertAlmostEqual(info["mass_kg"], 7.735, places=8)
        self.assertTrue(np.all(np.isfinite(info["reaction_force_body_n"])))
        self.assertTrue(np.all(np.isfinite(info["reaction_torque_body_nm"])))

    def test_offline_rl_supports_joint_and_end_effector_trajectories(self):
        for task in ("joint_trajectory", "ee_trajectory"):
            env = make_default_env(task)
            reset = env.reset(seed=19)
            observation = reset[0] if isinstance(reset, tuple) else reset
            self.assertEqual(observation.shape, (25,))
            action = env.hover_action()
            observation, reward, terminated, truncated, info = env.step(action)
            self.assertFalse(terminated)
            self.assertFalse(truncated)
            self.assertEqual(info["end_effector_position_m"].shape, (3,))
            self.assertEqual(info["end_effector_target_m"].shape, (3,))
            self.assertTrue(np.isfinite(info["end_effector_error_norm_m"]))

    def test_motor_thrust_evidence_template_cannot_promote_physics(self):
        frozen = json.loads(MOTOR_FREEZE_STATUS_PATH.read_text(encoding="utf-8"))
        template = json.loads(MOTOR_EVIDENCE_TEMPLATE_PATH.read_text(encoding="utf-8"))
        errors = validate_motor_thrust_evidence(template, frozen)
        self.assertTrue(errors)
        for motor in range(1, 9):
            self.assertTrue(
                any(error.startswith(f"motor {motor}:") for error in errors),
                f"template unexpectedly has no incomplete marker for motor {motor}",
            )

    def test_cad_axis_evidence_never_promotes_thrust_sign(self):
        axes = json.loads(MOTOR_AXIS_EVIDENCE_PATH.read_text(encoding="utf-8"))
        frozen = json.loads(MOTOR_FREEZE_STATUS_PATH.read_text(encoding="utf-8"))
        body = axes["body_frame_frozen"]
        self.assertEqual(
            body["positive_thrust_direction_status"],
            "UNRESOLVED_FOR_ALL_MOTORS",
        )
        self.assertNotIn("upward_thrust_motors", body)
        self.assertNotIn("downward_thrust_motors", body)
        raw_by_motor = {int(row["motor"]): row for row in axes["motors"]}
        for row in frozen["motors"]:
            motor = int(row["motor"])
            raw = raw_by_motor[motor]
            self.assertEqual(
                row["thrust_sign_status"],
                "UNRESOLVED_PROP_PITCH_OR_SIGNED_TEST_REQUIRED",
            )
            self.assertEqual(
                raw["thrust_sign_status"],
                "UNRESOLVED_PROP_PITCH_OR_SIGNED_TEST_REQUIRED",
            )
            np.testing.assert_allclose(
                row["cad_propeller_side_axis"],
                raw["cad_propeller_side_axis_frd"],
                atol=1.0e-6,
            )

    def test_propeller_instances_are_not_mirrored_by_assembly_transforms(self):
        pitch = json.loads(
            PROPELLER_PITCH_EVIDENCE_PATH.read_text(encoding="utf-8")
        )
        reuse = pitch["component_reuse_evidence"]
        self.assertTrue(reuse["all_instance_transform_determinants_positive"])
        self.assertEqual(reuse["mirrored_motor_instances"], [])
        self.assertEqual(
            sorted(int(row["motor"]) for row in pitch["motors"]),
            list(range(1, 9)),
        )
        for row in pitch["motors"]:
            self.assertAlmostEqual(
                row["assembly_transform_determinant"], 1.0, places=9
            )
            self.assertFalse(row["mirrored_instance"])
            self.assertFalse(row["pitch_sign_resolved"])

    def test_motor_freeze_table_separates_base_link_and_com_positions(self):
        frozen = json.loads(MOTOR_FREEZE_STATUS_PATH.read_text(encoding="utf-8"))
        formal = json.loads(CAD_V3_FLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        self.assertIn("base_link", frozen["coordinate_frame"])
        self.assertIn("provisional CAD-density-derived", frozen["formal_allocation_reference"])
        formal_by_motor = {int(row["motor"]): row for row in formal["rotors"]}
        com_distinct = False
        for row in frozen["motors"]:
            allocation = formal_by_motor[int(row["motor"])]
            np.testing.assert_allclose(
                row["position_m"], allocation["wrench_position_m"], atol=1.0e-6
            )
            com_distinct |= not np.allclose(
                row["position_m"], allocation["position_m"], atol=1.0e-6
            )
        self.assertTrue(com_distinct)

    def test_flight_configs_label_all_up_axes_as_hypotheses(self):
        for path in (CAD_V3_FLIGHT_CONFIG_PATH, CAD_V3_DEBUG_4KG_CONFIG_PATH):
            config = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(
                config["axis_assumption"],
                "ALL_UP_HYPOTHESIS_NOT_PHYSICAL_FACT",
            )
            self.assertEqual(
                config["positive_thrust_direction_status"],
                "UNRESOLVED_FOR_ALL_MOTORS",
            )
            for rotor in config["rotors"]:
                self.assertEqual(
                    rotor["thrust_sign_status"],
                    "UNRESOLVED_PROP_PITCH_OR_SIGNED_TEST_REQUIRED",
                )

    def test_complete_manufacturer_motor_evidence_closes_validator_gate(self):
        frozen = json.loads(MOTOR_FREEZE_STATUS_PATH.read_text(encoding="utf-8"))
        records = []
        for row in frozen["motors"]:
            motor = int(row["motor"])
            records.append(
                {
                    "motor": motor,
                    "rotation": row["rotation"],
                    "rotation_observation": "LOOKING_FROM_MOTOR_TOWARD_PROPELLER",
                    "prop_type": "REVERSE" if motor in {4, 5, 7, 8} else "NORMAL",
                    "thrust_sign_along_cad_propeller_ray": (
                        -1 if motor in {4, 5, 7, 8} else 1
                    ),
                    "maximum_thrust_n": 11.76798,
                    "evidence_type": "manufacturer",
                    "evidence_refs": [f"evidence/motor_{motor}_label.jpg"],
                    "propeller_part_number": f"TEST-PROP-{motor}",
                    "observation_definition": "Manufacturer declares handedness and axial thrust direction for this viewing convention.",
                }
            )
        self.assertEqual(
            validate_motor_thrust_evidence({"motors": records}, frozen), []
        )

    def test_bench_force_sign_must_match_declared_cad_ray_sign(self):
        frozen = json.loads(MOTOR_FREEZE_STATUS_PATH.read_text(encoding="utf-8"))
        records = []
        for row in frozen["motors"]:
            records.append(
                {
                    "motor": int(row["motor"]),
                    "rotation": row["rotation"],
                    "rotation_observation": "LOOKING_FROM_MOTOR_TOWARD_PROPELLER",
                    "prop_type": "NORMAL",
                    "thrust_sign_along_cad_propeller_ray": -1,
                    "maximum_thrust_n": 11.76798,
                    "evidence_type": "bench_test",
                    "evidence_refs": ["evidence/test.csv"],
                    "rpm": 1000,
                    "signed_axial_force_n": 0.5,
                    "force_positive_axis": "CAD_PROPELLER_SIDE_RAY",
                    "observation_definition": "Load-cell positive axis follows the CAD propeller-side ray.",
                }
            )
        errors = validate_motor_thrust_evidence({"motors": records}, frozen)
        for motor in range(1, 9):
            self.assertIn(
                f"motor {motor}: thrust sign contradicts signed axial force and force axis",
                errors,
            )


if __name__ == "__main__":
    unittest.main()
