import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription([
        # DeclareLaunchArgument(
        #     name='configuration',
        #     description='Tuning configuration to use. Must be name of file in config/ directory.'
        # ),
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
                        {'linear_slope': LaunchConfiguration('slope')}]
                # LaunchConfiguration('configuration')
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()