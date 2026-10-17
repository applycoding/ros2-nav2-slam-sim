# Copyright 2026 Arpan Bom
"""Launch slam_toolbox online async mapping with the indoor parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("indoor_robot_slam")
    slam_share = get_package_share_directory("slam_toolbox")
    default_params = os.path.join(share, "config", "mapper_params_online_async.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params,
                description="slam_toolbox parameter file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_share, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params_file,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )
