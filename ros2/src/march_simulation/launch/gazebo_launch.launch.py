#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Uses simulated time and publishes on /clock.'
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gzserver.launch.py')),
            launch_arguments=[('world',
                               PathJoinSubstitution([
                                   get_package_share_directory('march_simulation'),
                                   'worlds', 'march.world'
                               ])),
                              ('verbose', 'true'),
                              ('use_sim_time', use_sim_time),
                              ('pause', 'false')]
        )
    ])
