from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_arm_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "arm_zero_g.sdf"
    robot = package_share / "urdf" / "drone_with_arm_controlled.urdf"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": (
                f"-r -v 3 --physics-engine "
                f"gz-physics-bullet-featherstone-plugin {world}"
            )
        }.items(),
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
                    "arm_world",
                    "-name",
                    "my_drone",
                    "-file",
                    str(robot),
                    "-z",
                    "1.0",
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
                "/model/my_drone/joint_trajectory"
                "@trajectory_msgs/msg/JointTrajectory"
                "@gz.msgs.JointTrajectory"
            ),
            (
                "/world/arm_world/model/my_drone/joint_state"
                "@sensor_msgs/msg/JointState"
                "@gz.msgs.Model"
            ),
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                str(package_share),
            ),
            gazebo,
            bridge,
            spawn,
        ]
    )
