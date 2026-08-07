from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_arm_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "flight_world.sdf"
    robot = package_share / "urdf" / "drone_with_arm_controlled.urdf"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-r -v 3 --physics-engine ",
                LaunchConfiguration("physics_engine"),
                " ",
                str(world),
            ]
        }.items(),
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
                    "-x",
                    "0.25",
                    "-y",
                    "-0.20",
                    "-z",
                    "1.0",
                    "-R",
                    "0.0872665",
                    "-P",
                    "-0.0698132",
                    "-Y",
                    "0.0523599",
                ],
            )
        ],
    )
    controller = TimerAction(
        period=2.5,
        condition=IfCondition(LaunchConfiguration("enable_controller")),
        actions=[
            Node(
                package="drone_arm_sim",
                executable="gazebo_wrench_controller",
                output="screen",
                arguments=["--target", "0", "0", "1"],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_controller", default_value="true"),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-dartsim-plugin",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share)),
            gazebo,
            bridge,
            spawn,
            controller,
        ]
    )
