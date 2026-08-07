from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_arm_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "flight_world_250hz.sdf"
    robot = (
        package_share
        / "urdf"
        / "drone_with_arm_measured_flight_test.urdf"
    )
    config = package_share / "config" / "my_drone_measured_mounts.json"

    common_gz_args = (
        "-r -v 3 --physics-engine gz-physics-dartsim-plugin "
        f"{world}"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": common_gz_args}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-s {common_gz_args}"}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            (
                "/model/my_drone/odometry"
                "@nav_msgs/msg/Odometry"
                "[gz.msgs.Odometry"
            ),
            (
                "/world/flight_world/wrench"
                "@ros_gz_interfaces/msg/EntityWrench"
                "]gz.msgs.EntityWrench"
            ),
            (
                "/model/my_drone/command/motor_speed"
                "@actuator_msgs/msg/Actuators"
                "[gz.msgs.Actuators"
            ),
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world",
                    "flight_world",
                    "-name",
                    "my_drone",
                    "-file",
                    str(robot),
                    "-z",
                    LaunchConfiguration("spawn_z"),
                ],
            )
        ],
    )
    motor_model = TimerAction(
        # Start before spawning.  Both nodes wait for valid odometry, avoiding
        # a free-fall interval between entity creation and controller startup.
        period=0.5,
        actions=[
            Node(
                package="drone_arm_sim",
                executable="gazebo_direct_motor_model",
                output="screen",
                arguments=["--config", str(config)],
            )
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
                    "--urdf",
                    str(robot),
                    "--config",
                    str(config),
                    "--target",
                    "0",
                    "0",
                    LaunchConfiguration("target_ned_z"),
                    "--direct-thrust",
                ],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_z", default_value="1.0"),
            DeclareLaunchArgument("target_ned_z", default_value="-1.0"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("enable_controller", default_value="true"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share)),
            gazebo,
            gazebo_headless,
            bridge,
            spawn,
            motor_model,
            controller,
        ]
    )
