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
    world = package_share / "worlds" / "flight_world.sdf"
    robot = package_share / "urdf" / "my_drone_octorotor_example.urdf"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": (
                f"-r -v 3 --physics-engine "
                f"gz-physics-dartsim-plugin {world}"
            )
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": (
                f"-s -r -v 3 --physics-engine "
                f"gz-physics-dartsim-plugin {world}"
            )
        }.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )
    spawn = TimerAction(
        period=3.0,
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
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            (
                "/my_drone/command/motor_speed"
                "@actuator_msgs/msg/Actuators"
                "@gz.msgs.Actuators"
            ),
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
                "/world/flight_world/wrench/persistent"
                "@ros_gz_interfaces/msg/EntityWrench"
                "]gz.msgs.EntityWrench"
            ),
            (
                "/world/flight_world/wrench/clear"
                "@ros_gz_interfaces/msg/Entity"
                "]gz.msgs.Entity"
            ),
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_z", default_value="1.0"),
            DeclareLaunchArgument("headless", default_value="false"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share)),
            gazebo,
            gazebo_headless,
            bridge,
            spawn,
        ]
    )
