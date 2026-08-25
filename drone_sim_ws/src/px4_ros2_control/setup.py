from setuptools import find_packages, setup

package_name = "px4_ros2_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="my_drone simulation",
    maintainer_email="noreply@example.com",
    description="Safe ROS 2 DDS Offboard keyboard controller for PX4.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dds_wasd_control = px4_ros2_control.dds_wasd_control:main",
            "direct_xy_guardian = px4_ros2_control.direct_xy_guardian:main",
            "inner_loop_identification = px4_ros2_control.inner_loop_identification:main",
        ]
    },
)
