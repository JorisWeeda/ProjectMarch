import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='march_example_node',
            executable='example_node',
            output='screen',
            name='example_node')
    ])

