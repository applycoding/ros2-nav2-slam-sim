# Copyright 2026 Arpan Bom
"""Start Gazebo Classic with the indoor office and spawn the robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("gazebo_ros")
    world_share = get_package_share_directory("indoor_robot_gazebo")
    description_share = get_package_share_directory("indoor_robot_description")
    world_path = os.path.join(world_share, "worlds", "indoor_office.world")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    pause = LaunchConfiguration("pause")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("pause", default_value="false"),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-1.70",
                description="Spawn x in the map frame. Matches the lobby start pose.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-1.55",
                description="Spawn y in the map frame. Matches the lobby start pose.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.5708",
                description="Spawn yaw in radians. Faces the lobby door.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(description_share, "launch", "description.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": world_path,
                    "gui": gui,
                    "pause": pause,
                }.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_indoor_robot",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "indoor_robot",
                    "-x",
                    spawn_x,
                    "-y",
                    spawn_y,
                    "-z",
                    "0.01",
                    "-Y",
                    spawn_yaw,
                ],
            ),
        ]
    )
