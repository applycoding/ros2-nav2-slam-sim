# Copyright 2026 Arpan Bom
"""Launch Nav2 localization and navigation for the packaged office map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("indoor_robot_nav")
    bringup_share = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    default_map = os.path.join(nav_share, "maps", "indoor_office.yaml")
    default_rviz = os.path.join(nav_share, "rviz", "navigation.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameter file tuned for this robot.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Occupancy grid YAML. Defaults to the authored office map.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", default_rviz],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
