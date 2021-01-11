from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = LaunchConfiguration("parameter_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="parameter_file",
                default_value="example",
                description="Name of parameter file to load",
            ),
            Node(
                name="test_joint_gait_generator",
                package="march_test_joint_gait_generator",
                executable="generate_test_joint_gait",
                output="screen",
                parameters=[
                    [
                        PathJoinSubstitution(
                            [
                                get_package_share_directory(
                                    "march_test_joint_gait_generator"
                                ),
                                "config",
                                parameter_file,
                            ]
                        ),
                        ".yaml",
                    ]
                ],
            ),
        ]
    )
