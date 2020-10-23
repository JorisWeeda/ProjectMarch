import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    ld = launch.LaunchDescription([
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
        DeclareLaunchArgument(
            name='slope',
            default_value='100.0',
            description="The slope of the linear change in PID values. Only used when 'linear' is true"
        ),
        Node(
            package='march_gain_scheduling',
            executable='gain_scheduling_node',
            name='gain_scheduling_node',
            output='screen',
            parameters=[{'linearize_gain_scheduling': LaunchConfiguration('linear')},
                        {'linear_slope': LaunchConfiguration('slope')},
                        # Putting the path to the file and the string '.yaml' in a list makes sure the strings get
                        # concatenated after substitution.
                        # See: https://answers.ros.org/question/358655/ros2-concatenate-string-to-launchargument/
                        [PathJoinSubstitution([get_package_share_directory('march_gain_scheduling'), 'config',
                                               LaunchConfiguration('configuration')]), '.yaml']
                        ]
        )
    ])
    return ld

