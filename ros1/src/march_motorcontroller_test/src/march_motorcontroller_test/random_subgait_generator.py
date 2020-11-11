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
    version = 'final'

    gait_file_dir = 'motorcontroller-test'
    file_path_src = os.path.join(
        rospkg.RosPack().get_path('march_motorcontroller_test'), '..', '..',
        '..', '..', 'src', 'march_gait_files', gait_file_dir,
        gait_name, subgait_name, version + '.subgait')
    file_path_install = os.path.join(
        rospkg.RosPack().get_path('march_gait_files'), gait_file_dir,
        gait_name, subgait_name, version + '.subgait')

    duration = 1.5
    num_setpoints = 2

    subgait = generate_subgait(gait_name, subgait_name, version, duration,
                               num_setpoints)
    to_file(file_path_src, subgait)
    to_file(file_path_install, subgait)

    print('Generated yaml:\n')
    print(subgait.to_yaml())

def generate_subgait(gait_name, subgait_name, version, duration,
                     num_setpoints):
    urdf_file = 'test_joint_rotational'
    robot = urdf.Robot.from_xml_file(
        rospkg.RosPack().get_path('march_description') +
        '/urdf/{urdf_file}.urdf'.format(urdf_file=urdf_file))

    joint_name = 'rotational_joint'
    urdf_joint = JointTrajectory.get_joint_from_urdf(robot=robot,
                                                     joint_name=joint_name)
    limits = Limits.from_urdf_joint(urdf_joint)

    setpoints = generate_setpoints(duration, num_setpoints, limits, 'random')

    joints = [JointTrajectory.from_setpoints(name='rotational_joint',
                                             limits=limits,
                                             setpoints=setpoints,
                                             duration=duration)]
    subgait = Subgait(joints=joints,
                      duration=duration,
                      gait_type='',
                      gait_name=gait_name,
                      subgait_name=subgait_name,
                      version=version,
                      description='')

    return subgait


def generate_setpoints(duration, num_setpoints, limits, method='random'):
    setpoint_times = [(duration / num_setpoints) * i
                      for i in range(1, num_setpoints + 1)]
    setpoint_seconds_nanoseconds = [split_seconds_into_seconds_and_nanoseconds(
        time) for time in setpoint_times]

    positions = np.zeros(num_setpoints)
    if method == 'random':
        positions = np.random.uniform(limits.lower, limits.upper,
                                      num_setpoints)
    elif method == 'limits':
        positions = [limits.lower if i % 2 == 0 else limits.upper
                     for i in range(num_setpoints)]

    velocities = np.zeros(num_setpoints)

    setpoints = [{'time_from_start':
                      {'secs': setpoint_seconds_nanoseconds[i][0],
                       'nsecs': setpoint_seconds_nanoseconds[i][1]},
                  'position': positions[i],
                  'velocity': velocities[i]} for i in range(num_setpoints)]
    return setpoints


def split_seconds_into_seconds_and_nanoseconds(seconds):
    seconds_floored = int(math.floor(seconds))
    nanoseconds = (seconds - seconds_floored) * NANOSECONDS_PER_SECOND
    return seconds_floored, nanoseconds


def to_file(file_path, subgait):
    file = open(file_path, 'w')
    file.write(subgait.to_yaml())
    file.close()
