import math
import os

import rospkg
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.subgait import Subgait
import numpy as np
import rospy
from urdf_parser_py import urdf

NANOSECONDS_PER_SECOND = 1e9


def main():
    np.random.seed(123)

    gait_name = 'test_gait'
    subgait_name = 'perform_test'
    version = 'version_0'

    gait_file_dir = 'motorcontroller-test'

    file_path_src = os.path.join(
        rospkg.RosPack().get_path('march_motorcontroller_test'), '..', '..',
        '..', '..', 'src', 'march_gait_files', gait_file_dir,
        gait_name, subgait_name, version + '.subgait')
    file_path_install = os.path.join(
        rospkg.RosPack().get_path('march_gait_files'), gait_file_dir,
        gait_name, subgait_name, version + '.subgait')

    duration = 30
    num_setpoints = 20
    method = 'limits'

    # make limits x times stricter than defined by the urdf
    limits_factor = 0.95

    # Overriding urdf limits here
    limits = [-0.5, 0.5]

    subgait_generator = SubGaitGenerator(gait_name, subgait_name, version, duration,
                                         num_setpoints, method, limits=limits)
    subgait_generator.generate_subgait()
    subgait_generator.write_to_file(file_path_src)
    subgait_generator.write_to_file(file_path_install)

    print('Generated yaml:\n')
    print(subgait_generator.subgait.to_yaml())


class SubGaitGenerator:
    def __init__(self, gait_name, subgait_name, version, duration,
                 num_setpoints, method, limits_factor=1, limits=None,
                 description='', urdf_file='test_joint_rotational',
                 joint_name='rotational_joint'):
        """
        Initialize the subgait generator

        :param gait_name Name of the gait
        :param subgait_name Name of the subgait
        :param version Version of the gait
        :param duration Duration of the subgait in seconds
        :param num_setpoints Number of setpoints, all setpoint will be equal amount
               of time apart
        :param method Method used to generate the setpoints, either 'random' or 'limits'
        :param limits_factor Make limits x times stricter than defined by the urdf
        :param limits Optional parameter to override position limits of urdf
        :param description Description of the subgait
        :param urdf_file File to retrieve urdf from
        :param joint_name Name of the joint
        """
        self.gait_name = gait_name
        self.subgait_name = subgait_name
        self.version = version
        self.urdf_file = urdf_file
        self.duration = float(duration)
        self.num_setpoints = num_setpoints
        self.method = method
        self.description = description
        self.joint_name = joint_name
        robot = urdf.Robot.from_xml_file(
            rospkg.RosPack().get_path('march_description') +
            '/urdf/{urdf_file}.urdf'.format(urdf_file=self.urdf_file))

        urdf_joint = JointTrajectory.get_joint_from_urdf(robot=robot,
                                                         joint_name=self.joint_name)
        self.limits = Limits.from_urdf_joint(urdf_joint)
        if limits is not None:
            self.limits.lower = limits[0]
            self.limits.upper = limits[1]
        self.limits.lower *= limits_factor
        self.limits.upper *= limits_factor

        self.subgait = None

    def generate_subgait(self):
        setpoints = self.generate_setpoints()

        joints = [JointTrajectory.from_setpoints(name=self.joint_name,
                                                 limits=self.limits,
                                                 setpoints=setpoints,
                                                 duration=self.duration)]
        self.subgait = Subgait(joints=joints,
                          duration=self.duration,
                          gait_type='',
                          gait_name=self.gait_name,
                          subgait_name=self.subgait_name,
                          version=self.version,
                          description='')

    def generate_setpoints(self):
        setpoint_times = [(self.duration / (self.num_setpoints + 1)) * i
                          for i in range(1, self.num_setpoints + 1)]
        setpoint_seconds_nanoseconds = [split_seconds_into_seconds_and_nanoseconds(
            time) for time in setpoint_times]

        positions = np.zeros(self.num_setpoints)
        if self.method == 'random':
            positions = np.random.uniform(self.limits.lower, self.limits.upper,
                                          self.num_setpoints)
        elif self.method == 'limits':
            positions = [self.limits.lower if i % 2 == 0 else self.limits.upper
                         for i in range(self.num_setpoints)]

        velocities = np.zeros(self.num_setpoints)

        setpoints = [self.make_subgait_dict(0, 0, 0.0, 0.0)] + \
                    [self.make_subgait_dict(setpoint_seconds_nanoseconds[i][0],
                                            setpoint_seconds_nanoseconds[i][1],
                                            positions[i], velocities[i])
                     for i in range(self.num_setpoints)] + \
                    [self.make_subgait_dict(*split_seconds_into_seconds_and_nanoseconds(self.duration),
                                            position=0.0, velocity=0.0)]
        return setpoints

    def make_subgait_dict(self, secs, nsecs, position, velocity):
        return {'time_from_start':
                    {'secs': secs,
                     'nsecs': nsecs},
                'position': position,
                'velocity': velocity}

    def write_to_file(self, file_path):
        file = open(file_path, 'w')
        file.write(self.subgait.to_yaml())
        file.close()

def split_seconds_into_seconds_and_nanoseconds(seconds):
    seconds_floored = int(math.floor(seconds))
    nanoseconds = (seconds - seconds_floored) * NANOSECONDS_PER_SECOND
    return seconds_floored, nanoseconds