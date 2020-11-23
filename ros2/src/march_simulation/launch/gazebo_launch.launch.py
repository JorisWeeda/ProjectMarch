#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, \
    PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_ui = LaunchConfiguration('gazebo_ui')
    obstacle = LaunchConfiguration('obstacle')

    # This conditions evaluates to true if $(arg obstacle) != "none"
    obstacle_not_none = IfCondition(PythonExpression(['"true" if "none" != "',
                                                      obstacle, '" else "false"']))

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Uses simulated time and publishes on /clock.'
        ),
        DeclareLaunchArgument(
            name='gazebo_ui',
            default_value='true',
            description='Launches the Gazebo UI.'
        ),
        DeclareLaunchArgument(
            name='obstacle',
            default_value='none',
            description='Obstacle to load in the simulation.'
        ),
        # Launch both gazebo client and server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch',
                'gzserver.launch.py')),
            launch_arguments=[('world',
                              PathJoinSubstitution([
                                  get_package_share_directory('march_simulation'),
                                  'worlds', 'march.world'
                              ])),
                              ('verbose', 'true'),
                              ('use_sim_time', use_sim_time)]
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gzclient.launch.py')),
            launch_arguments=[('use_sim_time', use_sim_time),
                              ('verbose', 'true')],
            condition=IfCondition(gazebo_ui)
        ),
        # Spawn obstacle
        Node(
            package='march_simulation',
            executable='spawn_obstacle',
            parameters=[{'obstacle': obstacle}],
            output='screen',
            condition=obstacle_not_none
        )

    ])
