from glob import glob
import os

from setuptools import find_packages, setup


package_name = "drone_arm_sim"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.json") + glob("config/*.yaml"),
        ),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
        (
            os.path.join("share", package_name, "urdf", "my_drone_v2"),
            glob("urdf/my_drone_v2/*.urdf"),
        ),
        (
            os.path.join(
                "share", package_name, "meshes", "my_drone_v2", "visual"
            ),
            glob("meshes/my_drone_v2/visual/*.stl"),
        ),
        (
            os.path.join("share", package_name, "urdf", "meshes"),
            glob("meshes/*.stl"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="asus",
    maintainer_email="asus@example.com",
    description="SO-101 aerial manipulator simulation scaffold.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "allocation_analysis = drone_arm_sim.allocation_analysis:main",
            "arm_preset_control = drone_arm_sim.arm_preset_control:main",
            "floating_base_reaction = drone_arm_sim.floating_base_reaction:main",
            "flight_control_demo = drone_arm_sim.flight_control_demo:main",
            "gazebo_direct_motor_model = drone_arm_sim.gazebo_direct_motor_model:main",
            "gazebo_motor_controller = drone_arm_sim.gazebo_motor_controller:main",
            "gazebo_rotor_identification = drone_arm_sim.gazebo_rotor_identification:main",
            "gazebo_wrench_controller = drone_arm_sim.gazebo_wrench_controller:main",
            "hover_acceptance = drone_arm_sim.hover_acceptance:main",
            "inverse_kinematics = drone_arm_sim.inverse_kinematics:main",
            "model_analysis = drone_arm_sim.model_analysis:main",
            "motor_hover_demo = drone_arm_sim.motor_hover_demo:main",
            "trajectory_demo = drone_arm_sim.trajectory_demo:main",
        ],
    },
)
