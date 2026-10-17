# Copyright 2026 Arpan Bom
"""Start the waypoint mission runner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_robot_mission")
    default_waypoints = os.path.join(share, "config", "waypoints.yaml")
    default_params = os.path.join(share, "config", "mission_params.yaml")

    waypoints_file = LaunchConfiguration("waypoints_file")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints,
                description="YAML file of named map-frame waypoints.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="mission_runner parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock when true.",
            ),
            Node(
                package="indoor_robot_mission",
                executable="mission_runner",
                name="mission_runner",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "waypoints_file": waypoints_file,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
