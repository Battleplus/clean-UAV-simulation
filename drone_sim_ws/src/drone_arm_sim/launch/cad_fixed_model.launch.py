from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_arm_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "flight_world_250hz.sdf"
    robot = package_share / "urdf" / "my_drone_v2" / "my_drone_cad_fixed.urdf"
    common_args = f"-r -v 3 --physics-engine gz-physics-dartsim-plugin {world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": common_args}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-s {common_args}"}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "flight_world", "-name", "my_drone_v2",
                    "-file", str(robot), "-z", LaunchConfiguration("spawn_z"),
                ],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_z", default_value="0.35"),
            DeclareLaunchArgument("headless", default_value="false"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share)),
            gazebo,
            gazebo_headless,
            spawn,
        ]
    )
