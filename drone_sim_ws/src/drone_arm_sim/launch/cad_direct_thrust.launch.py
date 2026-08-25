import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_arm_sim"))
    motor_system_lib = Path(get_package_prefix("drone_motor_system")) / "lib"
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    # Keep the normal flight world as the default, while allowing a separate
    # contact/grasp world to be selected without changing the formal model.
    world = Path(os.environ.get(
        "MY_DRONE_WORLD",
        str(package_share / "worlds" / "flight_world_250hz.sdf"),
    ))
    robot = Path(os.environ.get(
        "MY_DRONE_URDF",
        str(package_share / "urdf" / "my_drone_v2" / "my_drone_cad_dynamic.urdf"),
    ))
    motion_reference = Path(os.environ.get(
        "SO101_MOTION_REFERENCE",
        str(package_share / "config" / "so101_motion_reference.json"),
    ))
    robot_xml = robot.read_text(encoding="utf-8").replace(
        "$(find drone_arm_sim)", str(package_share)
    )
    # Contact-only fixture: pin the CAD base kinematically while leaving all
    # SO101 joints under ros2_control.  This is selected explicitly by the
    # grasp regression and is never enabled by the PX4 / WASD startup path.
    if os.environ.get("MY_DRONE_KINEMATIC_BASE", "false").lower() in {
        "1", "true", "yes", "on"
    }:
        robot_xml = robot_xml.replace(
            "</robot>",
            "\n  <gazebo reference=\"base_link\">"
            "<kinematic>true</kinematic></gazebo>\n</robot>",
        )
    config = LaunchConfiguration("config_file")
    common_args = [
        "-r -v 3 --seed ",
        LaunchConfiguration("gz_seed"),
        f" --physics-engine gz-physics-dartsim-plugin {world}",
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": common_args}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-s ", *common_args]}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/model/my_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/flight_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            "/world/flight_world/wrench/latest@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            "/world/flight_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            "/world/flight_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity",
            "/my_drone/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators",
            "/world/flight_world/model/my_drone/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    robot_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_xml, "use_sim_time": True}],
    )
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "flight_world", "-name", "my_drone",
                    "-string", robot_xml, "-z", LaunchConfiguration("spawn_z"),
                ],
            )
        ],
    )
    motor_model = TimerAction(
        period=0.5,
        actions=[
            Node(
                package="drone_arm_sim",
                executable="gazebo_direct_motor_model",
                output="screen",
                arguments=[
                    "--config", config,
                    "--entity-name", "base_link",
                    "--command-topic", LaunchConfiguration("motor_command_topic"),
                    "--reaction-moment-ratio-m",
                    LaunchConfiguration("reaction_moment_ratio_m"),
                    "--wind-enu",
                    LaunchConfiguration("wind_enu_x"),
                    LaunchConfiguration("wind_enu_y"),
                    LaunchConfiguration("wind_enu_z"),
                    "--battery-dynamics-enabled",
                    LaunchConfiguration("battery_dynamics_enabled"),
                    "--battery-internal-resistance-ohm",
                    LaunchConfiguration("battery_internal_resistance_ohm"),
                    "--battery-capacity-ah",
                    LaunchConfiguration("battery_capacity_ah"),
                    "--battery-full-voltage-v",
                    LaunchConfiguration("battery_full_voltage_v"),
                    "--battery-empty-voltage-v",
                    LaunchConfiguration("battery_empty_voltage_v"),
                    "--battery-minimum-loaded-voltage-v",
                    LaunchConfiguration("battery_minimum_loaded_voltage_v"),
                    "--battery-thrust-voltage-exponent",
                    LaunchConfiguration("battery_thrust_voltage_exponent"),
                    "--arm-torque-feedforward-enabled",
                    LaunchConfiguration("arm_torque_feedforward_enabled"),
                    "--arm-reaction-torque-feedforward-gain",
                    LaunchConfiguration("arm_reaction_torque_feedforward_gain"),
                    "--arm-torque-feedforward-max-delta-n",
                    LaunchConfiguration("arm_torque_feedforward_max_delta_n"),
                    "--arm-static-com-feedforward-gain",
                    LaunchConfiguration("arm_static_com_feedforward_gain"),
                    "--arm-static-com-feedforward-time-constant-s",
                    LaunchConfiguration("arm_static_com_feedforward_time_constant_s"),
                    "--arm-disturbance-observer-enabled",
                    LaunchConfiguration("arm_disturbance_observer_enabled"),
                    "--arm-disturbance-observer-gain",
                    LaunchConfiguration("arm_disturbance_observer_gain"),
                    "--arm-disturbance-observer-max-delta-n",
                    LaunchConfiguration("arm_disturbance_observer_max_delta_n"),
                ],
            )
        ],
    )
    sensor_delay = Node(
        package="drone_arm_sim",
        executable="gazebo_sensor_delay",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_sensor_delay")),
        arguments=[
            "--imu-delay-ms", LaunchConfiguration("imu_delay_ms"),
            "--mag-delay-ms", LaunchConfiguration("mag_delay_ms"),
            "--baro-delay-ms", LaunchConfiguration("baro_delay_ms"),
            "--navsat-delay-ms", LaunchConfiguration("navsat_delay_ms"),
        ],
    )
    controller = TimerAction(
        period=0.5,
        condition=IfCondition(LaunchConfiguration("enable_controller")),
        actions=[
            Node(
                package="drone_arm_sim",
                executable="gazebo_motor_controller",
                output="screen",
                arguments=[
                    "--urdf", str(robot), "--config", config,
                    "--target", "0", "0", LaunchConfiguration("target_ned_z"),
                    "--direct-thrust",
                ],
            )
        ],
    )
    joint_state_controller = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration("enable_arm_control")),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                ],
            )
        ],
    )
    arm_controller = TimerAction(
        period=8.0,
        condition=IfCondition(LaunchConfiguration("enable_arm_control")),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "arm_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                ],
            )
        ],
    )
    arm_coupling_monitor = TimerAction(
        period=10.0,
        condition=IfCondition(LaunchConfiguration("enable_arm_control")),
        actions=[
            Node(
                package="drone_arm_sim",
                executable="arm_coupling_monitor",
                output="screen",
                arguments=[
                    "--urdf", str(robot),
                    "--motion-reference",
                    str(motion_reference),
                    "--rate-hz", LaunchConfiguration("arm_coupling_rate_hz"),
                    "--target-mass-kg", LaunchConfiguration("arm_coupling_target_mass_kg"),
                    "--payload-mass-kg", LaunchConfiguration("arm_payload_mass_kg"),
                    "--feedforward-limit-m-s2",
                    LaunchConfiguration("arm_feedforward_limit_m_s2"),
                ],
            )
        ],
    )
    arm_disturbance_observer = TimerAction(
        period=10.5,
        condition=IfCondition(
            LaunchConfiguration("arm_disturbance_observer_enabled")
        ),
        actions=[
            Node(
                package="drone_arm_sim",
                executable="arm_disturbance_observer",
                output="screen",
                arguments=[
                    "--config", config,
                    "--maximum-torque-nm",
                    LaunchConfiguration("arm_disturbance_observer_max_torque_nm"),
                ],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_z", default_value="1.0"),
            DeclareLaunchArgument("target_ned_z", default_value="-1.0"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("gz_seed", default_value="4027"),
            DeclareLaunchArgument("enable_controller", default_value="true"),
            DeclareLaunchArgument("enable_arm_control", default_value="false"),
            # PX4 publishes on /my_drone/command/motor_speed.  The optional
            # built-in hover controller publishes on /model/my_drone/...;
            # contact fixtures can select that topic without changing the
            # normal PX4 / WASD startup path.
            DeclareLaunchArgument(
                "motor_command_topic",
                default_value="/my_drone/command/motor_speed",
            ),
            DeclareLaunchArgument("reaction_moment_ratio_m", default_value="-1"),
            DeclareLaunchArgument("wind_enu_x", default_value="nan"),
            DeclareLaunchArgument("wind_enu_y", default_value="nan"),
            DeclareLaunchArgument("wind_enu_z", default_value="nan"),
            DeclareLaunchArgument("imu_delay_ms", default_value="4"),
            DeclareLaunchArgument("mag_delay_ms", default_value="10"),
            DeclareLaunchArgument("baro_delay_ms", default_value="20"),
            DeclareLaunchArgument("navsat_delay_ms", default_value="50"),
            DeclareLaunchArgument("enable_sensor_delay", default_value="true"),
            DeclareLaunchArgument("battery_dynamics_enabled", default_value="false"),
            DeclareLaunchArgument("battery_internal_resistance_ohm", default_value="nan"),
            DeclareLaunchArgument("battery_capacity_ah", default_value="nan"),
            DeclareLaunchArgument("battery_full_voltage_v", default_value="nan"),
            DeclareLaunchArgument("battery_empty_voltage_v", default_value="nan"),
            DeclareLaunchArgument("battery_minimum_loaded_voltage_v", default_value="nan"),
            DeclareLaunchArgument("battery_thrust_voltage_exponent", default_value="nan"),
            DeclareLaunchArgument(
                "arm_torque_feedforward_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "arm_reaction_torque_feedforward_gain", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "arm_torque_feedforward_max_delta_n", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "arm_static_com_feedforward_gain", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "arm_static_com_feedforward_time_constant_s", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "arm_disturbance_observer_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "arm_disturbance_observer_gain", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "arm_disturbance_observer_max_torque_nm", default_value="0.08"
            ),
            DeclareLaunchArgument(
                "arm_disturbance_observer_max_delta_n", default_value="1.0"
            ),
            DeclareLaunchArgument("arm_coupling_rate_hz", default_value="3.0"),
            DeclareLaunchArgument("arm_coupling_target_mass_kg", default_value="7.735"),
            DeclareLaunchArgument("arm_payload_mass_kg", default_value="0.0"),
            DeclareLaunchArgument("arm_feedforward_limit_m_s2", default_value="0.6"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "my_drone_v2_cad.json"),
            ),
            # model://drone_arm_sim/... is resolved by searching for the
            # drone_arm_sim directory below each resource root.  Therefore
            # the root must be share/, not share/drone_arm_sim/ itself.
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share.parent)),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                str(motor_system_lib)
                + os.pathsep
                + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ),
            gazebo,
            gazebo_headless,
            bridge,
            robot_description_publisher,
            spawn,
            sensor_delay,
            motor_model,
            controller,
            joint_state_controller,
            arm_controller,
            arm_coupling_monitor,
            arm_disturbance_observer,
        ]
    )
