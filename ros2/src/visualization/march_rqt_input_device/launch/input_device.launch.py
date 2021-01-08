from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, \
    PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to launch rqt input device.
    :argument: use_sim_time, whether the node should use the simulation time as published on the /clock topic.
    :argument: ping_safety_node, whether the node should regularly send an Alive message for the safety node.
    """
    button_layout_file = [PathJoinSubstitution([
        get_package_share_directory('march_rqt_input_device'),
        'config',
        LaunchConfiguration('button_layout_file')
    ]), '.txt']

    return LaunchDescription([
        DeclareLaunchArgument(
            name='node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        DeclareLaunchArgument(
            name='ping_safety_node',
            default_value='True',
            description='Whether to ping the safety node'),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Whether to use simulation time'),
        DeclareLaunchArgument(
            name='button_layout_file',
            default_value='layout',
            description='Layout file to load.'
        ),
        Node(
            package='march_rqt_input_device',
            executable='input_device',
            output='screen',
            name='input_device',
            namespace='march',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'ping_safety_node': LaunchConfiguration('ping_safety_node')},
                {'button_layout_file': button_layout_file}
            ]
        )
    ])
