import math
from copy import deepcopy
from march_gait_selection.state_machine.home_gait import HomeGait
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait
from rclpy.duration import Duration


class SemiDynamicSetpointsGait(SetpointsGait):

    def __init__(self, gait_name, subgaits, graph):
        super(SemiDynamicSetpointsGait, self).__init__(f'dynamic_{gait_name}', subgaits, graph)
        self._should_freeze = False
        self._is_frozen = False
        self._freeze_duration = 0

    @property
    def can_freeze(self) -> bool:
        """Returns whether the gait has the ability to freeze. This is meant
        for noticing the step height and ending the subgait earlier. This is
        therefore not possible during the first second of the subgait, to
        prevent accidental freezing."""
        if self._time_since_start < 1 or self._should_freeze or self._is_frozen:
            return False
        return True

    def freeze(self, position: dict = {}, duration: float = 10.0):
        """
        If the subgait can freeze it will freeze for the given duration, this
        will later be changed to start the next subgait more dynamically
        after the short freeze
        :param position: The position to freeze in
        :param duration: How long to freeze in the current position
        """
        self._should_freeze = True
        self._freeze_position = position
        self._freeze_duration = duration

    def update(self, elapsed_time, logger):
        """
        Update the progress of the gait, should be called regularly.
        If the current subgait is still running, this does nothing.
        If the gait should be stopped, this will be done
        If the current subgait is done, it will start the next subgait
        :param elapsed_time:
        :return: trjectory, is_finished
        """
        self._time_since_start += elapsed_time
        # logger.info(f'Should freeze = {self._should_freeze}, is frozen = {self._is_frozen}, time since start = {self._time_since_start}')
        if self._should_freeze:
            logger.info(f'Executing freeze, time since start = {self._time_since_start}')
            trajectory = self._execute_freeze(logger)
            self._time_since_start = 0.0
            return trajectory, False

        if self._time_since_start < self._current_subgait.duration:
            return None, False

        if self._is_frozen:
            logger.info(f'After freeze')
            self._current_subgait = self._subgait_after_freeze
            trajectory = self._current_subgait.to_joint_trajectory_msg()
            self._time_since_start = 0.0  # New subgait is started, so reset the time
            self._is_frozen = False
            return trajectory, False

        return self._update_next_subgait()

    def _execute_freeze(self, logger):
        """
        Freezes the subgait, currently this means that there is a new subgait
        started of the given freeze duration which ends at the current position.
        If this happens in the middle of a subgait, it plans the rest of the
        original subgait after the freeze.
        :return:
        """
        self._previous_subgait = self._current_subgait.subgait_name
        logger.info(f'Previous is {self._current_subgait.subgait_name}')
        self._subgait_after_freeze = self.subgait_after_freeze(logger)
        self._current_subgait = self._freeze_subgait(logger)
        self._should_freeze = False
        self._is_frozen = True
        logger.info(f'Frozen, current = {self._current_subgait.subgait_name}, next={self._subgait_after_freeze.subgait_name}')
        return self._current_subgait.to_joint_trajectory_msg()

    def subgait_after_freeze(self, logger):
        """
        Generates the subgait that should be executed after the freezing.
        :return: The subgait to execute after the freeze
        """
        if self._time_since_start < self._current_subgait.duration:
            subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in subgait_after_freeze:
                joint.from_begin_point(self._time_since_start, self._freeze_position[joint.name], logger)
            subgait_after_freeze.duration -= self._time_since_start
            subgait_after_freeze.subgait_name = 'dynamic_' + self._current_subgait.subgait_name
            self.graph._graph[subgait_after_freeze.subgait_name] = {'to':
                self.graph[(self._previous_subgait, self.graph.TO)]}
            logger.info(f"Current subgait after: duration={self._current_subgait.duration} {[self._current_subgait.get_joint(joint.name).setpoints for joint in self._current_subgait.joints]}")
            logger.info(f"After freeze:  duration={subgait_after_freeze.duration}  {[subgait_after_freeze.get_joint(joint.name).setpoints for joint in self._current_subgait.joints]}")
        else:
            subgait_after_freeze = self.subgaits[self.graph[
                (self._previous_subgait, self.graph.TO)]]
        return subgait_after_freeze

    def _freeze_subgait(self, logger):
        """
        Generates a subgait of the freeze duration based on the current position.
        :return: A subgait to freeze in current position
        """
        position = self._position_after_time(self._time_since_start)
        logger.info(f'Position theory is: {position}')
        logger.info(f'Position reality is: {self._freeze_position}')
        position = self._freeze_position
        new_dict = {
            'description': 'A subgait that stays in the same position',
            'duration': {
                'nsecs': Duration(seconds=self._freeze_duration).nanoseconds,
                'secs': math.floor(self._freeze_duration),
            },
            'gait_type': 'walk_like',
            'joints': dict([(joint.name, [{
                'position': position[joint.name],
                'time_from_start': {
                    'nsecs': (duration - math.floor(duration)) * 1e9,
                    'secs': math.floor(duration),
                },
                'velocity': 0}
                for duration in [self._freeze_duration]])
                            for joint in self._current_subgait.joints]),
            'name': 'dynamic_freeze',
            'version': 'Only version',
        }
        logger.info(f"{[new_dict['joints'][joint.name][0] for joint in self._current_subgait.joints]}")
        freeze_subgait = Subgait.from_dict(robot=self._current_subgait.robot,
                                           subgait_dict=new_dict,
                                           gait_name=self.gait_name,
                                           subgait_name='dynamic_freeze',
                                           version='Only version, generated from code')
        return freeze_subgait

    def _position_after_time(self, elapsed_time):
        """
        The position that the exoskeleton should be in after a certain elapsed
        time of the current subgait.
        :param elapsed_time: The time since the start of the subgait in secs.
        :return: dict with joints and joint positions
        """
        return {joint.name: joint.get_interpolated_setpoint(elapsed_time).position for
                joint in self._current_subgait}
