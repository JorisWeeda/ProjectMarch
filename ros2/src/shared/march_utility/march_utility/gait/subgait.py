"""Defines a base class for subgaits that can be executed by the exoskeleton."""
from __future__ import annotations
import os
import re
import math
from typing import List, Tuple
from rclpy.duration import Duration
from trajectory_msgs import msg as trajectory_msg
import yaml
from urdf_parser_py import urdf
from march_utility.exceptions.gait_exceptions import (
    NonValidGaitContent,
    SubgaitInterpolationError,
    GaitError,
)
from march_utility.foot_classes.feet_state import FeetState
from march_utility.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
    weighted_average_floats,
)
from .joint_trajectory import JointTrajectory
from .limits import Limits
from .setpoint import Setpoint

PARAMETRIC_GAITS_PREFIX = "_pg_"
NANOSEC_TO_SEC = 1e-9
SUBGAIT_SUFFIX = ".subgait"
JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()


class Subgait(object):
    """Base class for usage of the defined subgaits."""

    joint_class = JointTrajectory

    def __init__(
        self,
        joints: List[JointTrajectory],
        duration: Duration,
        gait_type: str = "walk_like",
        gait_name: str = "Walk",
        subgait_name: str = "right_open",
        version: str = "First try",
        description: str = "Just a simple gait",
        robot: urdf.Robot = None,
    ) -> None:
        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name
        self.robot = robot
        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    # region Create subgait
    @classmethod
    def from_file(cls, robot: urdf.Robot, file_name: str) -> Subgait:
        """
        Extract sub gait data of the given yaml.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name:
            The .yaml file name of the subgait

        :returns
            A populated Subgait object
        """
        if robot is None:
            raise TypeError("Robot is None, should be a valid urdf.Robot object")
        if file_name is None:
            raise TypeError("Filename is None, should be a string")

        with open(file_name, "r") as yaml_file:
            subgait_dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)

        gait_name = file_name.split("/")[-3]
        subgait_name = file_name.split("/")[-2]
        version = file_name.split("/")[-1].replace(SUBGAIT_SUFFIX, "")

        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version)

    @classmethod
    def from_name_and_version(
        cls,
        robot: urdf.Robot,
        gait_dir: str,
        gait_name: str,
        subgait_name: str,
        version: str,
    ) -> Subgait:
        """
        Load subgait based from file(s) based on name and version.

        :param robot: The robot corresponding to the given subgait file
        :param gait_dir: The directory with all the gaits
        :param gait_name: The name of the corresponding gait
        :param subgait_name: The name of the subgait to load
        :param version: The version to use, this can be parametric
        :return: A populated Subgait object.
        """
        subgait_path = os.path.join(gait_dir, gait_name, subgait_name)
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, parameter = Subgait.unpack_parametric_version(
                version
            )
            base_version_path = os.path.join(
                subgait_path, base_version + SUBGAIT_SUFFIX
            )
            if base_version == other_version:
                return cls.from_file(robot, base_version_path)
            else:
                other_version_path = os.path.join(
                    subgait_path, other_version + SUBGAIT_SUFFIX
                )
                return cls.from_files_interpolated(
                    robot,
                    base_version_path,
                    other_version_path,
                    parameter,
                    use_foot_position=True,
                )

        else:
            subgait_version_path = os.path.join(subgait_path, version + SUBGAIT_SUFFIX)
            return cls.from_file(robot, subgait_version_path)

    @classmethod
    def from_files_interpolated(
        cls,
        robot: urdf.Robot,
        file_name_base: str,
        file_name_other: str,
        parameter: float,
        use_foot_position: bool = False,
    ) -> Subgait:
        """
        Extract two subgaits from files and interpolate.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name_base:
            The .yaml file name of the base subgait
        :param file_name_other:
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1
        :param use_foot_position:
            Determine whether the interpolation should be done on the foot location or
            on the joint angles

        :return:
            A populated Subgait object
        """
        base_subgait = cls.from_file(robot, file_name_base)
        other_subgait = cls.from_file(robot, file_name_other)
        return cls.interpolate_subgaits(
            base_subgait, other_subgait, parameter, use_foot_position
        )

    @classmethod
    def from_dict(
        cls,
        robot: urdf.Robot,
        subgait_dict: dict,
        gait_name: str,
        subgait_name: str,
        version: str,
    ):
        """
        List parameters from the yaml file in organized lists.

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
            raise GaitError("Cannot create gait without a loaded robot.")

        duration = Duration(nanoseconds=subgait_dict["duration"])
        joint_list = []
        for name, points in sorted(
            subgait_dict["joints"].items(), key=lambda item: item[0]
        ):
            urdf_joint = cls.joint_class.get_joint_from_urdf(robot, name)
            if urdf_joint is None or urdf_joint.type == "fixed":
                continue
            limits = Limits.from_urdf_joint(urdf_joint)
            joint_list.append(
                cls.joint_class.from_setpoints(name, limits, points, duration)
            )
        subgait_type = (
            subgait_dict["gait_type"] if subgait_dict.get("gait_type") else ""
        )
        subgait_description = (
            subgait_dict["description"] if subgait_dict.get("description") else ""
        )

        return cls(
            joint_list,
            duration,
            subgait_type,
            gait_name,
            subgait_name,
            version,
            subgait_description,
            robot,
        )

    # endregion

    # region Create messages
    def to_joint_trajectory_msg(self):
        """
        Create trajectory msg for the publisher.

        :returns
            a ROS msg for the joint trajectory
        """
        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        joint_trajectory_msg.joint_names = [joint.name for joint in self.joints]

        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = Duration(
                seconds=timestamp
            ).to_msg()

            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    # endregion

    # region Validate subgait
    def validate_subgait_transition(self, next_subgait: Subgait) -> bool:
        """
        Validate the trajectory transition of this gait to a given gait.

        :param next_subgait:
            The subgait subsequently to this gait (not the previous one!)

        :returns:
            True if trajectory transition correct else False
        """
        from_subgait_joint_names = set(self.get_joint_names())
        to_subgait_joint_names = set(next_subgait.get_joint_names())

        if from_subgait_joint_names != to_subgait_joint_names:
            raise NonValidGaitContent(
                msg="Gait {gait}, structure of joints does not match between "
                "subgait {fn} and subgait {tn}".format(
                    gait=self.gait_name,
                    fn=self.subgait_name,
                    tn=next_subgait.subgait_name,
                )
            )

        for joint_name in to_subgait_joint_names:
            from_joint = self.get_joint(joint_name)
            to_joint = next_subgait.get_joint(joint_name)
            if not from_joint.validate_joint_transition(to_joint):
                return False

        return True

    # endregion

    # region Manipulate subgait
    def scale_timestamps_subgait(
        self, new_duration: float, rescale: bool = True
    ) -> None:
        """Scale or cut off all the setpoint to match the duration in both subgaits.

        :param new_duration: the new duration to scale the setpoints with.
        :param rescale:
            set to true if all points should be rescaled, alternative is cut off
            after new duration.

        """
        new_duration = round(new_duration, Setpoint.digits)

        for joint in self.joints:
            joint.set_duration(new_duration, rescale)
        self.duration = new_duration

    def create_interpolated_setpoints(self, timestamps: List[float]) -> None:
        """Equalize the setpoints of the subgait match the given timestamps.

        :param timestamps: the new timestamps to use when creating the setpoints
        """
        timestamps = sorted(set(timestamps + self.get_unique_timestamps()))

        for joint in self.joints:
            new_joint_setpoints = []
            for timestamp in timestamps:
                if timestamp > self.duration:
                    raise IndexError(
                        f"Gait {self.gait_name}, subgait {self.subgait_name} could "
                        f"not extrapolate timestamp outside max duration"
                    )

                new_joint_setpoints.append(joint.get_interpolated_setpoint(timestamp))

            joint.setpoints = new_joint_setpoints

    @classmethod
    def interpolate_subgaits(
        cls,
        base_subgait: Subgait,
        other_subgait: Subgait,
        parameter: float,
        use_foot_position: bool = False,
    ) -> Subgait:
        """
        Linearly interpolate two subgaits with the parameter to get a new subgait.

        :param base_subgait:
            base subgait, return value if parameter is equal to zero
        :param other_subgait:
            other subgait, return value if parameter is equal to one
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1
        :param use_foot_position:
            Determine whether the interpolation should be done on the foot
            location or on the joint angles

        :return:
            The interpolated subgait
        """
        if parameter == 1:
            return other_subgait
        if parameter == 0:
            return base_subgait
        if not (0 < parameter < 1):
            raise ValueError(
                f"Parameter for interpolation should be in the interval [0, 1], "
                f"but is {parameter}"
            )

        if use_foot_position:
            joints = Subgait.get_foot_position_interpolated_joint_trajectories(
                base_subgait, other_subgait, parameter
            )
        else:
            joints = Subgait.get_joint_angle_interpolated_joint_trajectories(
                base_subgait, other_subgait, parameter
            )

        description = (
            f"Interpolation between base version {base_subgait.version}, "
            f"and other version {other_subgait.version} with parameter {parameter}. "
            f"Based on foot position: {use_foot_position}"
        )

        duration = weighted_average_floats(
            base_subgait.duration, other_subgait.duration, parameter
        )

        gait_type = (
            base_subgait.gait_type if parameter <= 0.5 else other_subgait.gait_type
        )
        version = "{0}{1}_({2})_({3})".format(
            PARAMETRIC_GAITS_PREFIX,
            parameter,
            base_subgait.version,
            other_subgait.version,
        )
        return Subgait(
            joints,
            duration,
            gait_type,
            base_subgait.gait_name,
            base_subgait.subgait_name,
            version,
            description,
        )

    # endregion

    # region Get functions
    def get_unique_timestamps(self) -> List[float]:
        """Get the timestamps that are unique to a setpoint."""
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name: str) -> JointTrajectory:
        """Get joint object with given name or index."""
        return next(joint for joint in self.joints if joint.name == name)

    def get_joint_names(self) -> List[str]:
        """Get the names of all the joints existing in the joint list."""
        return [joint.name for joint in self.joints]

    @property
    def starting_position(self) -> dict:
        """Get a dictionary of joint positions at the start of this subgait."""
        return {joint.name: joint.setpoints[0].position for joint in self.joints}

    @property
    def final_position(self) -> dict:
        """Get a dictionary of joint positions at the end of this subgait."""
        return {joint.name: joint.setpoints[-1].position for joint in self.joints}

    # endregion

    def to_dict(self) -> dict:
        """Get the subgait represented as a dictionary."""
        duration = Duration(seconds=self.duration)
        return {
            "description": self.description,
            "duration": {
                "nsecs": duration.nanoseconds,
                "secs": math.floor(self.duration),
            },
            "gait_type": self.gait_type,
            "joints": dict(
                [
                    (
                        joint.name,
                        [
                            {
                                "position": setpoint.position,
                                "time_from_start": {
                                    "nsecs": Duration(
                                        seconds=setpoint.time
                                    ).nanoseconds,
                                    "secs": int(setpoint.time),
                                },
                                "velocity": setpoint.velocity,
                            }
                            for setpoint in joint.setpoints
                        ],
                    )
                    for joint in self.joints
                ]
            ),
            "name": self.subgait_name,
            "version": self.version,
        }

    def to_yaml(self) -> str:
        """Return a YAML string representation of the subgait."""
        return yaml.dump(self.to_dict())

    # region Class methods
    def __getitem__(self, index):
        return self.joints[index]

    def __len__(self):
        return len(self.joints)

    # endregion

    @staticmethod
    def validate_version(gait_path: str, subgait_name: str, version: str) -> bool:
        """Check whether a gait exists for the gait.

        :param gait_path: The path to the gait
        :param subgait_name: The name of the subgait
        :param version: The version of the subgait
        :return: Whether the gait is valid
        """
        subgait_path = os.path.join(gait_path, subgait_name)
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            Subgait.validate_parametric_version(subgait_path, version)
        else:
            version_path = os.path.join(subgait_path, version + SUBGAIT_SUFFIX)
            if not os.path.isfile(version_path):
                return False
        return True

    @staticmethod
    def validate_parametric_version(subgait_path: str, version: str) -> bool:
        """Check whether a parametric gait is valid.

        :param subgait_path: The path to the subgait file
        :param version: The version of the parametric gait
        :return: True if the subgait is parametrized between two existing subgaits
        """
        base_version, other_version, _ = Subgait.unpack_parametric_version(version)
        base_version_path = os.path.join(subgait_path, base_version + SUBGAIT_SUFFIX)
        other_version_path = os.path.join(subgait_path, other_version + SUBGAIT_SUFFIX)
        return all(
            not os.path.isfile(version_path)
            for version_path in [base_version_path, other_version_path]
        )

    @staticmethod
    def unpack_parametric_version(version: str) -> Tuple[str, str, float]:
        """Unpack a version to base version, other version and parameter."""
        parameter_search = re.search(
            r"^{0}(\d+\.\d+)_".format(PARAMETRIC_GAITS_PREFIX), version
        )
        if parameter_search is None:
            raise SubgaitInterpolationError(
                "Parametric version was stored in wrong " "format."
            )
        parameter = float(parameter_search.group(1))
        versions = re.findall(r"\([^\)]*\)", version)
        base_version = versions[0][1:-1]
        other_version = versions[1][1:-1]
        return base_version, other_version, parameter

    @staticmethod
    def check_foot_position_interpolation_is_safe(
        base_subgait: Subgait, other_subgait: Subgait
    ) -> None:
        """Check whether two subgaits are safe to be interpolated on foot location."""
        number_of_setpoints = len(base_subgait.joints[0].setpoints)
        for base_joint, other_joint in zip(
            sorted(base_subgait.joints, key=lambda joint: joint.name),
            sorted(other_subgait.joints, key=lambda joint: joint.name),
        ):
            JointTrajectory.check_joint_interpolation_is_safe(
                base_joint, other_joint, number_of_setpoints
            )

    @staticmethod
    def get_foot_position_interpolated_joint_trajectories(
        base_subgait: Subgait, other_subgait: Subgait, parameter: float
    ) -> List[JointTrajectory]:
        """Create a list of joint trajectories by interpolating foot locations.

        The foot location corresponding to the resulting trajectories is equal
        to the weighted average (with the  parameter) of the foot locations
        corresponding to the base and other subgait.

        :param base_subgait: base subgait, return value if parameter is equal to zero
        :param other_subgait: other subgait, return value if parameter is equal to one
        :param parameter: Parameter for interpolation, between 0 and 1
        :return: A list of interpolated joint trajectories
        """
        interpolated_joint_trajectories = []
        # for inverse kinematics it is required that all joints have the same
        # number of setpoints as to calculate the foot position at a certain time.
        # The inverse kinematics also needs acces to the 'ith' setpoints of all joints
        Subgait.check_foot_position_interpolation_is_safe(base_subgait, other_subgait)
        number_of_setpoints = len(base_subgait.joints[0].setpoints)
        (
            base_setpoints_to_interpolate,
            other_setpoints_to_interpolate,
        ) = Subgait.change_order_of_joints_and_setpoints(base_subgait, other_subgait)
        new_setpoints: dict = {joint.name: [] for joint in base_subgait.joints}
        # fill all joints in new_setpoints except the ankle joints using
        # the inverse kinematics
        for setpoint_index in range(0, number_of_setpoints):
            base_feet_state = FeetState.from_setpoints(
                base_setpoints_to_interpolate[setpoint_index]
            )
            other_feet_state = FeetState.from_setpoints(
                other_setpoints_to_interpolate[setpoint_index]
            )
            new_feet_state = FeetState.weighted_average_states(
                base_feet_state, other_feet_state, parameter
            )
            setpoints_to_add = FeetState.feet_state_to_setpoints(new_feet_state)
            for joint_name in JOINT_NAMES_IK:
                new_setpoints[joint_name].append(setpoints_to_add[joint_name])

        # fill the ankle joint using the angle based linear interpolation
        for ankle_joint in ["left_ankle", "right_ankle"]:
            for base_setpoint, other_setpoint in zip(
                base_subgait.get_joint(ankle_joint).setpoints,
                other_subgait.get_joint(ankle_joint).setpoints,
            ):
                new_ankle_setpoint_to_add = Setpoint.interpolate_setpoints(
                    base_setpoint, other_setpoint, parameter
                )
                new_setpoints[ankle_joint].append(new_ankle_setpoint_to_add)

        duration = weighted_average_floats(
            base_subgait.duration, other_subgait.duration, parameter
        )

        for joint in base_subgait.joints:
            interpolated_joint_trajectory_to_add = JointTrajectory(
                joint.name, joint.limits, new_setpoints[joint.name], duration
            )
            interpolated_joint_trajectories.append(interpolated_joint_trajectory_to_add)

        return interpolated_joint_trajectories

    @staticmethod
    def get_joint_angle_interpolated_joint_trajectories(
        base_subgait: Subgait, other_subgait: Subgait, parameter: float
    ) -> List[JointTrajectory]:
        """Interpolate joint trajectories for each joint trajectory in two subgaits.

        :param base_subgait: base subgait, return value if parameter is equal to zero
        :param other_subgait: other subgait, return value if parameter is equal to one
        :param parameter: Parameter for interpolation, between 0 and 1
        :return: A list of linearly interpolated joint trajectories
        """
        interpolated_joint_trajectories = []
        for base_joint, other_joint in zip(
            sorted(base_subgait.joints, key=lambda joint: joint.name),
            sorted(other_subgait.joints, key=lambda joint: joint.name),
        ):
            if base_joint.name != other_joint.name:
                raise SubgaitInterpolationError(
                    "The subgaits to interpolate do not have the same joints, base"
                    " subgait has {0}, while other subgait has {1}".format(
                        base_joint.name, other_joint.name
                    )
                )
            interpolated_joint_trajectory_to_add = (
                JointTrajectory.interpolate_joint_trajectories(
                    base_joint, other_joint, parameter
                )
            )
            interpolated_joint_trajectories.append(interpolated_joint_trajectory_to_add)

        return interpolated_joint_trajectories

    @staticmethod
    def change_order_of_joints_and_setpoints(
        base_subgait: Subgait, other_subgait: Subgait
    ) -> Tuple[List[dict], List[dict]]:
        """Change structure of joint trajectories to see positions per time point.

        This function goes over each joint to get needed setpoints
        (all first setpoints, all second setpoints..).
        These are placed in list with the correct index, where each entry contains a
        dictionary with joint name setpoint pairs. Also checks whether the joint
        trajectories are safe to interpolate.

        :param base_subgait: one of the subgaits to reorder
        :param other_subgait: the other subgait to reorder
        :return: The interpolated trajectory
        """
        number_of_setpoints = len(base_subgait.joints[0].setpoints)
        base_setpoints_to_interpolate: List[dict] = [
            {} for _ in range(number_of_setpoints)
        ]
        other_setpoints_to_interpolate: List[dict] = [
            {} for _ in range(number_of_setpoints)
        ]
        for setpoint_index in range(number_of_setpoints):
            for base_joint, other_joint in zip(
                sorted(base_subgait.joints, key=lambda joint: joint.name),
                sorted(other_subgait.joints, key=lambda joint: joint.name),
            ):
                base_setpoints_to_interpolate[setpoint_index][
                    base_joint.name
                ] = base_joint.setpoints[setpoint_index]
                other_setpoints_to_interpolate[setpoint_index][
                    other_joint.name
                ] = other_joint.setpoints[setpoint_index]
        return base_setpoints_to_interpolate, other_setpoints_to_interpolate
