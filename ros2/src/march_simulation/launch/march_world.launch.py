#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot = LaunchConfiguration('robot')
    ground_gait = LaunchConfiguration('ground_gait')
    gazebo_ui = LaunchConfiguration('gazebo_ui')
    fixed = LaunchConfiguration('fixed')


    xacro_path = PathJoinSubstitution([
        get_package_share_directory('march_description'), 'urdf', robot])

    # Override effort values as Gazebo uses different units than the actual IMC.
    # In theory, if the controller is properly tuned, these values can be arbitrarily large.
    # However, to limit the safety controller, we need to limit them.
    xacro_args = [" k_velocity_value_hfe:=", "60.0",
                  " k_velocity_value_kfe:=", "60.0",
                  " k_velocity_value_haa:=", "60.0 ",
                  " k_velocity_value_adpf:=", "15.0",
                  " k_position_value_hfe:=", "5000.0 ",
                  " k_position_value_kfe:=", "5000.0",
                  " k_position_value_haa:=", "5000.0 ",
                  " k_position_value_adpf:=", "5000.0",
                  " max_effort_rotary:=", "200.0 ",
                  " max_effort_linear:=", "200.0",
                  " ground_gait:=", ground_gait]
    robot_description = Command(['xacro', ' ', xacro_path, '.xacro'] +
                                xacro_args)
    #, ' ', xacro_args, ground_gait])


    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Uses simulated time and publishes on /clock.'
        ),
        DeclareLaunchArgument(
            name='fake_sensor_data',
            default_value='false',
            description='Publishes fake sensor data.'
        ),
        DeclareLaunchArgument(
            name='gazebo_ui',
            default_value='true',
            description='Launches the Gazebo UI.'
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false',
            description='Starts gazebo debugging with gdb.'
        ),
        DeclareLaunchArgument(
            name='fixed',
            default_value='true',
            description='Fixes the exoskeleton in the world'
        ),
        DeclareLaunchArgument(
            name='ground_gait',
            default_value='false',
            description='Exoskeleton will ground gait if true.'
        ),
        DeclareLaunchArgument(
            name='obstacle',
            default_value='none',
            description='Obstacle to load in the simulation.'
        ),
        DeclareLaunchArgument(
            name='robot',
            default_value='march4',
            description='The robot to run. Can be: march3, march4, test_joint_rotational.'
        ),
        DeclareLaunchArgument(
            name='controller',
            default_value='effort_control',
            description='Changes the controller used by simulation.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]),
        Node(
            package='march_simulation',
            executable='to_world_transform',
            name='world_transformer',
            output='screen',
            condition=UnlessCondition(fixed)
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            arguments=['-topic', 'robot_description',
                       '-entity', 'march'],
            output='screen'
        ),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #     os.path.join(get_package_share_directory('march_simulation'),
        #                  'launch', 'gazebo_launch.launch.py')),
        #     launch_arguments=[('use_sim_time', use_sim_time),
        #                       ('gazebo_ui', gazebo_ui)]
        # )
    ])
