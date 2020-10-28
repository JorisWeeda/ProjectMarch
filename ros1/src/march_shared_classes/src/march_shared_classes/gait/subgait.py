import os
import re

import rospy
from trajectory_msgs import msg as trajectory_msg
import yaml

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent, SubgaitInterpolationError
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError

from .joint_trajectory import JointTrajectory
from .limits import Limits
from .setpoint import Setpoint

PARAMETRIC_GAITS_PREFIX = '_pg_'


class Subgait(object):
    """Base class for usage of the defined subgaits."""

    joint_class = JointTrajectory

    def __init__(self, joints, duration, gait_type='walk_like', gait_name='Walk', subgait_name='right_open',
                 version='First try', description='Just a simple gait'):

        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name

        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    # region Create subgait
    @classmethod
    def from_file(cls, robot, file_name, *args):
        """Extract sub gait data of the given yaml.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name:
            The .yaml file name of the subgait

        :returns
            A populated Subgait object
        """
        if file_name is None or not os.path.isfile(file_name):
            raise FileNotFoundError(file_path=file_name)
        try:
            gait_name = file_name.split('/')[-3]
            subgait_name = file_name.split('/')[-2]
            version = file_name.split('/')[-1].replace('.subgait', '')

            with open(file_name, 'r') as yaml_file:
                subgait_dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)

        except Exception as e:
            rospy.logerr('Error occurred in subgait: {te}, {er} '.format(te=type(e), er=e))
            return None

        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version, *args)

    @classmethod
    def from_name_and_version(cls, robot, gait_dir, gait_name, subgait_name, version, *args):
        """Load subgait based from file(s) based on name and version.

        :param robot: The robot corresponding to the given subgait file
        :param gait_dir: The directory with all the gaits
        :param gait_name: The name of the corresponding gait
        :param subgait_name: The name of the subgait to load
        :param version: The version to use, this can be parametric
        :param args:
        :return: A populated Subgait object.
        """
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, parameter = Subgait.unpack_parametric_version(version)
            if base_version == other_version:
                subgait_path = os.path.join(gait_dir, gait_name, subgait_name, base_version + '.subgait')
                return cls.from_file(robot, subgait_path, *args)
            else:
                base_path = os.path.join(gait_dir, gait_name, subgait_name, base_version + '.subgait')
                other_path = os.path.join(gait_dir, gait_name, subgait_name, other_version + '.subgait')
                return cls.from_files_interpolated(robot, base_path, other_path, parameter, foot_pos=True)
        else:
            subgait_path = os.path.join(gait_dir, gait_name, subgait_name, version + '.subgait')
            return cls.from_file(robot, subgait_path, *args)

    @classmethod
    def from_files_interpolated(cls, robot, file_name_base, file_name_other, parameter, foot_pos=False, *args):
        """Extract two subgaits from files and interpolate.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name_base:
            The file name of the base subgait
        :param file_name_other:
            the file name of the oter subgait
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1
        :param foot_pos:
            Determine whether the interpolation should be done on the foot location or on the joint angles

        :return:
            A populated Subgait object
        """
        base_subgait = cls.from_file(robot, file_name_base, *args)
        other_subgait = cls.from_file(robot, file_name_other, *args)
        return cls.interpolate_subgaits(base_subgait, other_subgait, parameter, foot_pos)

    @classmethod
    def from_dict(cls, robot, subgait_dict, gait_name, subgait_name, version, *args):
        """List parameters from the yaml file in organized lists.

        :param robot:
            The robot corresponding to the given sub-gait file
        :param subgait_dict:
            The dictionary extracted from the yaml file
        :param gait_name:
            The name of the parent gait
        :param subgait_name:
            The name of the child (sub)gait
        :param version:
            The version of the yaml file

        :returns
            A populated Subgait object
        """
        if robot is None:
            rospy.logerr('Cannot create gait without a loaded robot.')
            return None

        duration = rospy.Duration(subgait_dict['duration']['secs'], subgait_dict['duration']['nsecs']).to_sec()

        joint_list = []
        for name, points in sorted(subgait_dict['joints'].items(), key=lambda item: item[0]):
            urdf_joint = cls.joint_class.get_joint_from_urdf(robot, name)
            if urdf_joint is None or urdf_joint.type == 'fixed':
                rospy.logwarn('Joint {0} is not in the robot description. Skipping joint.')
                continue
            limits = Limits.from_urdf_joint(urdf_joint)
            joint_list.append(cls.joint_class.from_setpoints(name, limits, points, duration, *args))
        subgait_type = subgait_dict['gait_type'] if subgait_dict.get('gait_type') else ''
        subgait_description = subgait_dict['description'] if subgait_dict.get('description') else ''

        return cls(joint_list, duration, subgait_type, gait_name, subgait_name, version, subgait_description)

    # endregion

    # region Create messages
    def to_joint_trajectory_msg(self):
        """Create trajectory msg for the publisher.

        :returns
            a ROS msg for the joint trajectory
        """
        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        joint_trajectory_msg.joint_names = [joint.name for joint in self.joints]

        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration.from_sec(timestamp)

            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)
                if interpolated_setpoint.time != timestamp:
                    rospy.logwarn('Time mismatch in joint {jn} at timestamp {ts}, '
                                  'got time {ti}'.format(jn=joint.name, ts=timestamp, ti=interpolated_setpoint.time))

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    # endregion

    # region Validate subgait
    def validate_subgait_transition(self, next_subgait):
        """Validate the trajectory transition of this gait to a given gait.

        :param next_subgait:
            The subgait subsequently to this gait (not the previous one!)

        :returns:
            True if trajectory transition correct else False
        """
        from_subgait_joint_names = set(self.get_joint_names())
        to_subgait_joint_names = set(next_subgait.get_joint_names())

        if from_subgait_joint_names != to_subgait_joint_names:
            raise NonValidGaitContent(msg='Gait {gait}, structure of joints does not match between '
                                          'subgait {fn} and subgait {tn}'.format(gait=self.gait_name,
                                                                                 fn=self.subgait_name,
                                                                                 tn=next_subgait.subgait_name))

        for joint_name in to_subgait_joint_names:
            from_joint = self.get_joint(joint_name)
            to_joint = next_subgait.get_joint(joint_name)
            if not from_joint.validate_joint_transition(to_joint):
                return False

        return True

    # endregion

    # region Manipulate subgait
    def scale_timestamps_subgait(self, new_duration, rescale=True):
        """Scale or cut off all the setpoint to match the duration in both subgaits.

        :param new_duration: the new duration to scale the setpoints with
        :param rescale: set to true if all points should be rescaled, alternative is cut off after new duration
        """
        new_duration = round(new_duration, Setpoint.digits)

        for joint in self.joints:
            joint.set_duration(new_duration, rescale)
        self.duration = new_duration

    def create_interpolated_setpoints(self, timestamps):
        """Equalize the setpoints of the subgait match the given timestamps.

        :param timestamps: the new timestamps to use when creating the setpoints
        """
        timestamps = sorted(set(timestamps + self.get_unique_timestamps()))

        for joint in self.joints:
            new_joint_setpoints = []
            for timestamp in timestamps:
                if timestamp > self.duration:
                    raise IndexError('Gait {gait}, subgait {subgait} could not extrapolate timestamp outside max '
                                     'duration'.format(gait=self.gait_name, subgait=self.subgait_name))

                new_joint_setpoints.append(joint.get_interpolated_setpoint(timestamp))

            joint.setpoints = new_joint_setpoints

    @classmethod
    def interpolate_subgaits(cls, base_subgait, other_subgait, parameter, foot_pos=False):
        """Linearly interpolate two subgaits with the parameter to get a new subgait. based on foot_pos, or on angles.

        :param base_subgait:
            base subgait, return value if parameter is equal to zero
        :param other_subgait:
            other subgait, return value if parameter is equal to one
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1
        :param foot_pos:
            Determine whether the interpolation should be done on the foot location or on the joint angles

        :return:
            The interpolated subgait
        """
        if parameter == 1:
            return other_subgait
        if parameter == 0:
            return base_subgait
        if not (0 < parameter < 1):
            raise ValueError('Parameter for interpolation should be in the interval [0, 1], but is {0}'
                             .format(parameter))

        if sorted(base_subgait.get_joint_names()) != sorted(other_subgait.get_joint_names()):
            raise SubgaitInterpolationError('The subgaits to interpolate do not have the same joints, base'
                                            ' subgait has {0}, while other subgait has {1}'.
                                            format(sorted(base_subgait.get_joint_names()),
                                                   sorted(other_subgait.get_joint_names())))

        joints = []
        num_setpoints = 0
        try:
            for base_joint in base_subgait.joints:
                other_joint = other_subgait.get_joint(base_joint.name)
                if other_joint is None:
                    raise SubgaitInterpolationError('Could not find a matching joint for base joint with name {0}.'.
                                                    format(base_joint.name))
                if foot_pos:
                    if num_setpoints == 0:
                        num_setpoints = len(base_joint._setpoints)
                        compare_joint_name = base_joint.name

                    if len(base_joint._setpoints) != num_setpoints:
                        raise SubgaitInterpolationError('Number of setpoints differ in subgait {0} {1} from {0} {2}.'.
                                                        format(base_subgait.subgait_name, base_joint.name,
                                                               compare_joint_name))
                    elif len(other_joint._setpoints) != num_setpoints:
                        raise SubgaitInterpolationError('Number of setpoint differs in subgait {0} {1} from {2} {3}.'.
                                                        format(other_subgait.subgait_name, base_joint.name,
                                                               base_subgait.subgait_name, compare_joint_name))
                else:
                    joints.append(cls.joint_class.interpolate_joint_trajectories(base_joint, other_joint, parameter))

        except SubgaitInterpolationError as e:
            raise e

        duration = base_subgait.duration * parameter + (1 - parameter) * other_subgait.duration

        if foot_pos:
            new_setpoints = {'left_hip_aa': [],
                             'left_hip_fe': [],
                             'left_knee': [],
                             'right_hip_aa': [],
                             'right_hip_fe': [],
                             'right_knee': [],
                             'left_ankle': [],
                             'right_ankle': []}
            # go over each joint to get needed setpoints (all first setpoints, all second setpoints..). These are needed
            # as calcuating the foot position requires the position of all joints at a certain time.
            for current_setpoints_index in range(0, num_setpoints):
                base_setpoints_to_interpolate = {}
                other_setpoints_to_interpolate = {}
                for base_joint in base_subgait.joints:
                    other_joint = other_subgait.get_joint(base_joint.name)
                    base_joint_setpoints = base_joint.setpoints
                    other_joint_setpoints = other_joint.setpoints
                    base_setpoints_to_interpolate[base_joint.name] = base_joint_setpoints[current_setpoints_index]
                    other_setpoints_to_interpolate[other_joint.name] = other_joint_setpoints[current_setpoints_index]
                interpolated_setpoints = Setpoint.interpolate_setpoints_position(base_setpoints_to_interpolate,
                                                                                 other_setpoints_to_interpolate,
                                                                                 parameter)
                for base_joint in base_subgait.joints:
                    new_setpoints[base_joint.name].append(interpolated_setpoints[base_joint.name])

            for base_joint in base_subgait.joints:
                joints.append(cls.joint_class(base_joint.name, base_joint.limits, new_setpoints[base_joint.name],
                                              duration))

        description = 'Interpolation between base version {0}, and other version {1} with parameter{2}'.format(
            base_subgait.version, other_subgait.version, parameter)

        gait_type = base_subgait.gait_type if parameter <= 0.5 else other_subgait.gait_type
        version = '{0}{1}_({2})_({3})'.format(PARAMETRIC_GAITS_PREFIX, parameter, base_subgait.version,
                                              other_subgait.version)
        return Subgait(joints, duration, gait_type, base_subgait.gait_name, base_subgait.subgait_name, version,
                       description)
    # endregion

    # region Get functions
    def get_unique_timestamps(self):
        """The timestamp that is unique to a setpoint."""
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        """Get joint object with given name or index."""
        return next(joint for joint in self.joints if joint.name == name)

    def get_joint_names(self):
        """Get the names of all the joints existing in the joint list."""
        return [joint.name for joint in self.joints]

    @property
    def starting_position(self):
        """Returns a dictionary of joint positions at the start of this subgait."""
        return {joint.name: joint.setpoints[0].position for joint in self.joints}

    @property
    def final_position(self):
        """Returns a dictionary of joint positions at the end of this subgait."""
        return {joint.name: joint.setpoints[-1].position for joint in self.joints}

    # endregion

    def to_yaml(self):
        """Returns a YAML string representation of the subgait."""
        duration = rospy.Duration.from_sec(self.duration)
        output = {
            'description': self.description,
            'duration': {
                'nsecs': duration.nsecs,
                'secs': duration.secs,
            },
            'gait_type': self.gait_type,
            'joints': dict([(joint.name, [{
                'position': setpoint.position,
                'time_from_start': {
                    'nsecs': rospy.Duration.from_sec(setpoint.time).nsecs,
                    'secs': int(setpoint.time),
                },
                'velocity': setpoint.velocity}
                for setpoint in joint.setpoints]) for joint in self.joints]),
            'name': self.subgait_name,
            'version': self.version,
        }
        return yaml.dump(output)

    # region Class methods
    def __getitem__(self, index):
        return self.joints[index]

    def __len__(self):
        return len(self.joints)
    # endregion

    @staticmethod
    def validate_version(gait_path, subgait_name, version):
        """Check whether a gait exists for the gait."""
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, _ = Subgait.unpack_parametric_version(version)
            base_subgait_path = os.path.join(gait_path, subgait_name, base_version + '.subgait')
            other_subgait_path = os.path.join(gait_path, subgait_name, other_version + '.subgait')
            for subgait_path in [base_subgait_path, other_subgait_path]:
                if not os.path.isfile(subgait_path):
                    rospy.logwarn('{sp} does not exist'.format(sp=subgait_path))
                    return False
        else:
            subgait_path = os.path.join(gait_path, subgait_name, version + '.subgait')
            if not os.path.isfile(subgait_path):
                rospy.logwarn('{sp} does not exist'.format(sp=subgait_path))
                return False
        return True

    @staticmethod
    def unpack_parametric_version(version):
        """Unpack a version to base version, other version and parameter."""
        parameter_str = re.search(r'^{0}(\d+\.\d+)_'.format(PARAMETRIC_GAITS_PREFIX), version).group(1)
        parameter = float(parameter_str)
        versions = re.findall(r'\([^\)]*\)', version)
        base_version = versions[0][1:-1]
        other_version = versions[1][1:-1]
        return base_version, other_version, parameter
