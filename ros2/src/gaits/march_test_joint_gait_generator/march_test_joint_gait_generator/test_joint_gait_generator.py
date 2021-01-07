import math
import os
from typing import Tuple

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.subgait import Subgait
from rclpy.parameter import Parameter
from rclpy.node import Node
from urdf_parser_py import urdf

gait_files_package = 'march_gait_files'

gait_files_src_path = os.path.join(get_package_share_directory(gait_files_package),
                                        '..', '..', '..', '..',
                                        'src', 'gaits', gait_files_package)
gait_files_install_path = get_package_share_directory(gait_files_package)

DIRECTORIES = [gait_files_src_path, gait_files_install_path]

NANOSECONDS_PER_SECOND = 1e9

NODE_NAME = 'test_joint_gait_generator'

class TestJointGaitGenerator(Node):
    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self.gait_directory = self.get_parameter('gait_directory').get_parameter_value().string_value
        self.gait_name = self.get_parameter('gait_name').get_parameter_value().string_value
        self.subgait_name = self.get_parameter('subgait_name').get_parameter_value().string_value
        self.version = self.get_parameter('version').get_parameter_value().string_value

        # Convert parameter value from double to integer if necessary
        self.duration = self.get_float_parameter('duration')
        self.start_position = self.get_float_parameter('start_position')

        self.num_setpoints = self.get_parameter('num_setpoints').get_parameter_value().integer_value

        self.description = self.get_parameter('description').get_parameter_value().string_value
        if self.description == '':
            self.description = self.generate_description()

        self.robot_name = self.get_parameter('robot').get_parameter_value().string_value
        self.urdf_file = os.path.join(get_package_share_directory('march_description'),
                                      'urdf', f'{self.robot_name}.urdf')
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value

        self.robot = urdf.Robot.from_xml_file(self.urdf_file)
        self.joint = JointTrajectory.get_joint_from_urdf(robot=self.robot,
                                                         joint_name=self.joint_name)

        self.limits = Limits.from_urdf_joint(self.joint)
        if not self.get_parameter('use_urdf_limits').get_parameter_value().bool_value:
            self.limits.lower = self.get_float_parameter('lower_limit')
            self.limits.upper = self.get_float_parameter('upper_limit')

        limits_factor = self.get_float_parameter('limits_factor')
        self.limits.lower *= limits_factor
        self.limits.upper *= limits_factor

        self.directories = DIRECTORIES

    def generate_and_write_all(self):
        self.make_all_directories()
        default_yaml = self.generate_default_yaml()
        gait = self.generate_gait()
        subgait = self.generate_subgait()

        self.write_to_all_directories(default_yaml, 'default.yaml')
        self.write_to_all_directories(gait,
                                      os.path.join(self.gait_name, f'{self.gait_name}.gait'))
        self.write_to_all_directories(subgait,
                                      os.path.join(self.gait_name, self.subgait_name, f'{self.version}.subgait'))

        self.get_logger().info('Creating files done')

    def make_all_directories(self):
        """Make all necessary directories."""
        for directory in self.directories:
            self.make_subgait_directory(directory)

    def make_subgait_directory(self, gait_files_path):
        """Make the subgait directory in the gait directory.

        Also makes gait files directory in march_gait_files package, if necessary.

        :param gait_files_path Path to march_gait_files package.
                               Can be path to ros1 src, ros1 install or ros2 src folder
        """
        directory = os.path.join(gait_files_path, self.gait_directory, self.gait_name, self.subgait_name)
        if not os.path.isdir(directory):
            os.makedirs(directory)

    def generate_default_yaml(self):
        """Generate the default.yaml string for the gait."""
        default = {
            'gaits': {
                self.gait_name: {
                    self.subgait_name: self.version
                }
            },
            'positions': {
                'setup': {
                    'gait_type': '',
                    'joints': {
                        self.joint_name: self.start_position
                    }
                }
            }
        }
        return yaml.dump(default)


    def generate_gait(self):
        """Generate the gait yaml string"""
        gait = {
            'name': self.gait_name,
            'subgaits': {
                'start': {
                    'to': self.subgait_name
                },
                self.subgait_name: {
                    'to': 'end'
                }
            }
        }
        return yaml.dump(gait)

    def generate_subgait(self):
        """Generate the test_joint subgait yaml string"""
        setpoints = self.generate_setpoints()

        joints = [JointTrajectory.from_setpoints(name=self.joint_name,
                                                 limits=self.limits,
                                                 setpoints=setpoints,
                                                 duration=self.duration)]
        return Subgait(joints=joints,
                      duration=self.duration,
                      gait_type='',
                      gait_name=self.gait_name,
                      subgait_name=self.subgait_name,
                      version=self.version,
                      description=self.description).to_yaml()

    def generate_setpoints(self):
        """Generate the setpoints for the subgait."""
        setpoint_times_from_start = [
            (self.duration / (self.num_setpoints + 1)) * i
            for i in range(1, self.num_setpoints + 1)]
        setpoint_seconds_nanoseconds = [
            self.split_seconds_into_seconds_and_nanoseconds(time)
            for time in setpoint_times_from_start]
        positions = [self.limits.lower if i % 2 == 0
                     else self.limits.upper
                     for i in range(self.num_setpoints)]
        velocities = [0.0] * self.num_setpoints

        setpoints = [self.make_setpoint_dict(self.start_position, 0, 0.0, 0.0)] + \
                    [self.make_setpoint_dict(setpoint_seconds_nanoseconds[i][0],
                                            setpoint_seconds_nanoseconds[i][1],
                                            positions[i], velocities[i])
                     for i in range(self.num_setpoints)] + \
                    [self.make_setpoint_dict
                     (*self.split_seconds_into_seconds_and_nanoseconds(self.duration),
                      position=self.start_position, velocity=0.0)]
        return setpoints

    def generate_description(self):
        """Generate a description of the subgait."""
        return f'Subgait generated using TestJointGaitGenerator, with a ' \
               f'duration of {self.duration}s and {self.num_setpoints} setpoints.'

    @staticmethod
    def make_setpoint_dict(seconds: int, nanoseconds: int,
                           position: float, velocity: float) -> dict:
        """ Create a setpoint dictionary."""
        return {'time_from_start':
                    {'secs': seconds,
                     'nsecs': nanoseconds},
                'position': position,
                'velocity': velocity}

    def write_to_all_directories(self, content: str, relative_path: str):
        """Write the content to all directories.

        :param content Content to write.
        :param relative_path Path relative from the gait directory.
        """
        for directory in self.directories:
            TestJointGaitGenerator.write_to_file(
                content, os.path.join(directory, self.gait_directory, relative_path))

    @staticmethod
    def write_to_file(content, file_path):
        """Write content to a file."""
        file = open(file_path, 'w')
        file.write(content)
        file.close()

    @staticmethod
    def split_seconds_into_seconds_and_nanoseconds(seconds: float) -> Tuple[int, int]:
        """Split seconds into seconds and nanoseconds."""
        seconds_floored = int(math.floor(seconds))
        nanoseconds = int((seconds - seconds_floored) * NANOSECONDS_PER_SECOND)
        return seconds_floored, nanoseconds

    def get_float_parameter(self, name: str) -> float:
        parameter = self.get_parameter(name)
        if parameter.type_ == Parameter.Type.DOUBLE:
            return parameter.get_parameter_value().double_value
        else:
            return float(parameter.get_parameter_value().integer_value)

def main():
    rclpy.init()
    test_joint_gait_generator = TestJointGaitGenerator()
    test_joint_gait_generator.generate_and_write_all()