# Copyright 2026 Arpan Bom
"""Gazebo office plus slam_toolbox. Does not start Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    gazebo_share = get_package_share_directory("indoor_robot_gazebo")
    slam_share = get_package_share_directory("indoor_robot_slam")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "world.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time, "gui": gui}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "slam.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
