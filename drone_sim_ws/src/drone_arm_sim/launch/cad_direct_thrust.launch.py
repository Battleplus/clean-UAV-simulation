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
    robot = package_share / "urdf" / "my_drone_v2" / "my_drone_cad_dynamic.urdf"
    robot_xml = robot.read_text(encoding="utf-8").replace(
        "$(find drone_arm_sim)", str(package_share)
    )
    config = LaunchConfiguration("config_file")
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
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/model/my_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/flight_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
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
                    "--reaction-moment-ratio-m",
                    LaunchConfiguration("reaction_moment_ratio_m"),
                ],
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_z", default_value="1.0"),
            DeclareLaunchArgument("target_ned_z", default_value="-1.0"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("enable_controller", default_value="true"),
            DeclareLaunchArgument("enable_arm_control", default_value="false"),
            DeclareLaunchArgument("reaction_moment_ratio_m", default_value="-1"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "my_drone_v2_cad.json"),
            ),
            # model://drone_arm_sim/... is resolved by searching for the
            # drone_arm_sim directory below each resource root.  Therefore
            # the root must be share/, not share/drone_arm_sim/ itself.
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share.parent)),
            gazebo,
            gazebo_headless,
            bridge,
            robot_description_publisher,
            spawn,
            motor_model,
            controller,
            joint_state_controller,
            arm_controller,
        ]
    )
