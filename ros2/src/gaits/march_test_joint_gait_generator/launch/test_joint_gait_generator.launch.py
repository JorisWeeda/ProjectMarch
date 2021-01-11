from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gait_directory = LaunchConfiguration("gait_directory")
    gait_name = LaunchConfiguration("gait_name")
    subgait_name = LaunchConfiguration("subgait_name")
    version = LaunchConfiguration("version")
    num_setpoints = LaunchConfiguration("num_setpoints")
    duration = LaunchConfiguration("duration")
    robot = LaunchConfiguration("robot")
    joint_name = LaunchConfiguration("joint_name")
    description = LaunchConfiguration("description")
    start_position = LaunchConfiguration("start_position")
    use_urdf_limits = LaunchConfiguration("use_urdf_limits")
    limits_factor = LaunchConfiguration("limits_factor")
    lower_limit = LaunchConfiguration("lower_limit")
    upper_limit = LaunchConfiguration("upper_limit")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gait_directory",
                default_value="test_joint_gaits",
                description="Directory to place test joint gait in",
            ),
            DeclareLaunchArgument(
                name="gait_name",
                default_value="test_joint_gait",
                description="Name of test_joint gait",
            ),
            DeclareLaunchArgument(
                name="subgait_name",
                default_value="perform_test",
                description="Name of the single subgait in the gait",
            ),
            DeclareLaunchArgument(
                name="version",
                default_value="example",
                description="Version of the single test_joint subgait",
            ),
            DeclareLaunchArgument(
                name="description",
                default_value="",
                description="Description of the gait."
                "If left empty a description is generated.",
            ),
            DeclareLaunchArgument(
                name="num_setpoints",
                default_value="2",
                description="Number of setpoints in the gait. "
                "1 setpoint means the gait moves to one end and back to the middle"
                "2 setpoints means the gait moves to both ends and then to the middle etc. etc.",
            ),
            DeclareLaunchArgument(
                name="duration",
                default_value="6.0",
                description="Duration of the test_joint gait in seconds",
            ),
            DeclareLaunchArgument(
                name="use_urdf_limits",
                default_value="true",
                description="Whether to use the urdf limits.",
            ),
            DeclareLaunchArgument(
                name="limits_factor",
                default_value="0.95",
                description="Move joint from begin position to limit * limits_factor,"
                "This should has to be between 0 and 1.",
            ),
            DeclareLaunchArgument(
                name="lower_limit",
                default_value="0.0",
                description="Lower limit of the joint. Only used if use_urdf_limits is false.",
            ),
            DeclareLaunchArgument(
                name="upper_limit",
                default_value="0.0",
                description="Upper limit of the joint. Only used if use_urdf_limits is false.",
            ),
            DeclareLaunchArgument(
                name="robot",
                default_value="test_joint_rotational",
                description="Name of urdf robot file",
            ),
            DeclareLaunchArgument(
                name="joint_name",
                default_value="rotational_joint",
                description="Name of the joint in the urdf file.",
            ),
            DeclareLaunchArgument(
                name="start_position",
                default_value="0.0",
                description="Start position of the gait.",
            ),
            Node(
                name="test_joint_gait_generator",
                package="march_test_joint_gait_generator",
                executable="generate_test_joint_gait",
                output="screen",
                parameters=[
                    {"gait_directory": gait_directory},
                    {"gait_name": gait_name},
                    {"subgait_name": subgait_name},
                    {"version": version},
                    {"num_setpoints": num_setpoints},
                    {"duration": duration},
                    {"robot": robot},
                    {"joint_name": joint_name},
                    {"start_position": start_position},
                    {"description": description},
                    {"use_urdf_limits": use_urdf_limits},
                    {"limits_factor": limits_factor},
                    {"lower_limit": lower_limit},
                    {"upper_limit": upper_limit},
                ],
            ),
        ]
    )
