import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    # General arguments
    node_prefix = LaunchConfiguration('node_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot = LaunchConfiguration('robot')
    # Input device arguments
    rqt_input = LaunchConfiguration('rqt_input')
    ping_safety_node = LaunchConfiguration('ping_safety_node')
    # Robot state publisher arguments
    robot_state_publisher = LaunchConfiguration('robot_state_publisher')
    xacro_path = LaunchConfiguration('xacro_path')
    # Gait selection arguments
    gait_selection = LaunchConfiguration('gait_selection')
    gait_package = LaunchConfiguration('gait_package')
    gait_directory = LaunchConfiguration('gait_directory')
    # Simulation arguments
    gazebo_ui = LaunchConfiguration('gazebo_ui')
    fixed = LaunchConfiguration('fixed')
    ground_gait = LaunchConfiguration('ground_gait')
    obstacle = LaunchConfiguration('obstacle')
    controller = LaunchConfiguration('controller')
    simulation = LaunchConfiguration('simulation')

    return launch.LaunchDescription([
        # GENERAL ARGUMENTS
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether to use simulation time as published on the '
                        '/clock topic by gazebo instead of system time.'),
        DeclareLaunchArgument(
            name='robot',
            default_value='march4',
            description='Robot to use.'),
        # RQT INPUT DEVICE ARGUMENTS
        DeclareLaunchArgument(
            name='rqt_input',
            default_value='True',
            description='If this argument is false, the rqt input device will'
                        'not be launched.'),
        DeclareLaunchArgument(
            'ping_safety_node',
            default_value='True',
            description='Whether the input device should ping the safety node'
                        'with an alive message every 0.2 seconds'),
        # ROBOT STATE PUBLISHER ARGUMENTS
        DeclareLaunchArgument(
            name='robot_state_publisher',
            default_value='True',
            description='Whether or not to launch the robot state publisher,'
                        'this allows nodes to get the urdf and to subscribe to'
                        'potential urdf updates. This is necesary for gait selection'
                        'to be able to launch'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=os.path.join(get_package_share_directory('march_description'), 'urdf', 'march4.xacro'),
            description='Path to the <robot>.xacro file that should be used to'
                        'obtain the robot description that is published by the '
                        'robot state publisher'),
        # GAIT SELECTION ARGUMENTS
        DeclareLaunchArgument(
            name='gait_selection',
            default_value='True',
            description='Whether to launch the march gait selection node.'),
        DeclareLaunchArgument(
            'gait_package',
            default_value='march_gait_files',
            description='The package where the gait files are located.'),
        DeclareLaunchArgument(
            'gait_directory',
            default_value='training-v',
            description='The directory in which the gait files to use are located, '
                        'relative to the gait_package.'),
        # SIMULATION ARGUMENT
        DeclareLaunchArgument(
            name='gazebo_ui',
            default_value='true',
            description='Launches the Gazebo UI.'
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
            name='controller',
            default_value='effort_control',
            description='Changes the controller used by simulation.'
        ),
        DeclareLaunchArgument(
            name='simulation',
            default_value='false',
            description='Whether to launch the simulation'
        ),

        # Launch rqt input device if not rqt_input:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_rqt_input_device'), 'launch', 'input_device.launch.py')),
            launch_arguments=[('ping_safety_node', ping_safety_node),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(rqt_input)),
        # Launch robot state publisher (from march_description) if not robot_state_publisher:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_description'), 'launch', 'march_description.launch.py')),
            launch_arguments=[('xacro_path', xacro_path),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(robot_state_publisher)),
        # Launch march gait selection if not gait_selection:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_gait_selection'), 'launch', 'gait_selection.launch.py')),
            launch_arguments=[('gait_directory', gait_directory),
                              ('use_sim_time', use_sim_time),
                              ('gait_package', gait_package)],
            condition=IfCondition(gait_selection)),
        # Launch simulation
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_simulation'),
                         'launch', 'march_world.launch.py')),
            launch_arguments=[('gazebo_ui', gazebo_ui),
                              ('use_sim_time', use_sim_time),
                              ('fixed', fixed),
                              ('ground_gait', ground_gait),
                              ('obstacle', obstacle),
                              ('controller', controller),
                              ('robot', robot)],
            condition=IfCondition(simulation))
    ])
