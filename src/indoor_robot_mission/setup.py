import os

from setuptools import setup

package_name = "indoor_robot_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            ["launch/mission.launch.py"],
        ),
        (
            os.path.join("share", package_name, "config"),
            ["config/waypoints.yaml", "config/mission_params.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arpan Bom",
    maintainer_email="applycoding@gmail.com",
    description="Waypoint mission runner for indoor Nav2 navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_runner = indoor_robot_mission.mission_runner_node:main",
        ],
    },
)
