import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        DeclareLaunchArgument(
            'ping_safety_node',
            default_value='True',
            description='Whether to ping the safety node'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether to use simulation time'),
        DeclareLaunchArgument(
            name='rqt_input',
            default_value='True',
            description='Launches the rqt input device.'),
        DeclareLaunchArgument(
            name='gain_scheduling',
            default_value='True',
            description='Launches the gain scheduling node.'),
        DeclareLaunchArgument(
            name='configuration',
            default_value='groundgait',
            description='Tuning configuration to use. Must be name of file in config/ directory.'
        ),
        DeclareLaunchArgument(
            name='linear',
            default_value='true',
            description='Whether to linearize the change in PID values'
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_gain_scheduling'), 'launch',
                         'march_gain_scheduling.launch.py')),
            launch_arguments=[('linear', LaunchConfiguration('linear')),
                              ('configuration', LaunchConfiguration('configuration')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(LaunchConfiguration('gain_scheduling'))),
        # Launch rqt input device if not rqt_input:=false is given as argument
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_rqt_input_device'), 'launch', 'input_device.launch.py')),
            launch_arguments=[('node_prefix', LaunchConfiguration('node_prefix')),
                              ('ping_safety_node', LaunchConfiguration('ping_safety_node')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(LaunchConfiguration('rqt_input')))
    ])
