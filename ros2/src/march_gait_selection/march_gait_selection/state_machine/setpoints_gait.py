from copy import deepcopy

from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_gait_selection.state_machine.home_gait import HomeGait
from march_shared_classes.exceptions.gait_exceptions import GaitError
from march_shared_classes.gait.gait import Gait
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.gait.subgait import Subgait

from .gait_interface import GaitInterface
from .state_machine_input import TransitionRequest


class SetpointsGait(GaitInterface, Gait):
    """ The standard gait built up from setpoints """
    def __init__(self, gait_name, subgaits, graph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = 0.0
        ############################################
        self._should_freeze = False
        self._is_frozen = False
        self._freeze_duration = 0
        ############################################

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        if self._current_subgait is not None:
            return self._current_subgait.subgait_name
        else:
            return None

    @property
    def version(self):
        if self._current_subgait is not None:
            return self._current_subgait.version
        else:
            return None

    @property
    def duration(self):
        if self._current_subgait is not None:
            return self._current_subgait.duration
        else:
            return None

    @property
    def gait_type(self):
        if self._current_subgait is not None:
            return self._current_subgait.gait_type
        else:
            return None

    @property
    def starting_position(self):
        return self.subgaits[self.graph.start_subgaits()[0]].starting_position

    @property
    def final_position(self):
        return self.subgaits[self.graph.end_subgaits()[0]].final_position

    def start(self):
        """
        Start the gait, sets current subgait to the first subgait, resets the
        time and generates the first trajectory.
        :return: A JointTrajectory message with the trajectory of the first subgait.
        """
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = 0.0
        return self._current_subgait.to_joint_trajectory_msg()

    def update(self, elapsed_time):
        """
        Update the progress of the gait, should be called regularly.
        If the current subgait is still running, this does nothing.
        If the gait should be stopped, this will be done
        If the current subgait is done, it will start the next subgait
        :param elapsed_time:
        :return: trjectory, is_finished
        """
        self._time_since_start += elapsed_time
        if self._should_freeze:
            trajectory = self._execute_freeze()
            self._time_since_start = 0.0
            return trajectory, False

        if self._time_since_start < self._current_subgait.duration:
            return None, False

        if self._should_stop:
            next_subgait = self._stop()

        elif self._transition_to_subgait is not None and not self._is_transitioning:
            return self._transition_subgait()

        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self._transition_to_subgait.subgait_name
            self._transition_to_subgait = None
            self._is_transitioning = False

        elif self._is_frozen:
            self._current_subgait = self._subgait_after_freeze
            trajectory = self._current_subgait.to_joint_trajectory_msg()
            self._time_since_start = 0.0 # New subgait is started, so reset the time
            self._is_frozen = False
            return trajectory, False
        else:
            # If there is transition to do, go to next (TO) subgait
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]

        if next_subgait == self.graph.END:
            return None, True
        self._current_subgait = self.subgaits[next_subgait]
        trajectory = self._current_subgait.to_joint_trajectory_msg()
        self._time_since_start = 0.0 # New subgait is started, so reset the time
        return trajectory, False

    def _execute_freeze(self):
        ## Set the rest of current subgait as next subgait
        if self._time_since_start < self._current_subgait.duration:
            self._subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in self._subgait_after_freeze:
                joint.from_begin_point(self._time_since_start)
        ## Set a freeze subgait for the freeze duration
        freeze_subgait = deepcopy(self._current_subgait)
        freeze_subgait.duration = self._freeze_duration
        # new_joint_trajectories = []
        position = self._current_position(self._time_since_start)
        for joint in freeze_subgait.joints:
            joint.setpoints = [Setpoint(time=self._freeze_duration,
                                        position=position[joint.name],
                                        velocity=0)]
        self._current_subgait = freeze_subgait
        self._should_freeze = False
        self._is_frozen = True
        return freeze_subgait.to_joint_trajectory_msg()



    def transition(self, transition_request):
        """
        Request to transition between two subgaits with increasing or decreasing
        size of the step.
        :param transition_request:
        :return:
        """
        if self._is_transitioning or self._should_stop:
            return False

        if transition_request == TransitionRequest.DECREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.DECREASE_SIZE)]
        elif transition_request == TransitionRequest.INCREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.INCREASE_SIZE)]
        else:
            return False

        if name is not None:
            self._transition_to_subgait = self.subgaits[name]
            return True
        return False

    def stop(self):
        """ Called when the current gait should be stopped. Return a boolean
        for whether the stopping was succesfull. """
        if self.graph.is_stoppable() and not self._is_transitioning \
                and self._transition_to_subgait is None:
            self._should_stop = True
            return True
        else:
            return False

    def end(self):
        """Called when the gait has finished."""
        self._current_subgait = None

    def set_subgait_versions(self, robot, gait_directory, version_map):
        """
        Change the versions of the subgaits.
        :param robot: The robot model used.
        :param gait_directory: The directory where the gaits are located.
        :param version_map: The map with the new versions to use.
        """
        if self._current_subgait is None:
            super(SetpointsGait, self).set_subgait_versions(robot, gait_directory,
                                                            version_map)
        else:
            raise GaitError('Cannot change subgait version while gait is being executed')

    def _stop(self):
        next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.STOP)]
        if next_subgait is None:
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]
        else:
            self._should_stop = False
        return next_subgait

    def _transition_subgait(self):
        old_subgait = self.subgaits[self.graph[
            (self._current_subgait.subgait_name, self.graph.TO)]]
        new_subgait = self.subgaits[self.graph[
            (self._transition_to_subgait.subgait_name, self.graph.TO)]]
        transition_subgait = TransitionSubgait.from_subgaits(
            old_subgait, new_subgait, '{s}_transition'.format(
                s=self._transition_to_subgait.subgait_name))
        self._current_subgait = transition_subgait
        self._time_since_start = 0.0
        self._is_transitioning = True
        return transition_subgait.to_joint_trajectory_msg(), False

    def freeze(self, duration: float = 10.0):
        self._should_freeze = True
        self._freeze_duration = duration
        return True

    def _current_position(self, elapsed_time):
        return {joint.name: joint.get_interpolated_setpoint(elapsed_time).position for
                joint in self._current_subgait}
