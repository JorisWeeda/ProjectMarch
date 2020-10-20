from threading import Event
from typing import List

import rclpy
from march_shared_classes.exceptions.parameter_exceptions import ParameterGetException, ParameterSetException
from march_shared_msgs.msg import CurrentGait, GainValues, GainValuesList
from march_shared_msgs.srv import GetParamStringList, GetGainValuesList, SetGainValuesList
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer(Node):
    def __init__(self):
        """Initialize the DynamicPIDReconfigurer / gain_scheduling node."""
        super().__init__('gain_scheduling_node', automatically_declare_parameters_from_overrides=True)
        self._gait_type = None
        self._joint_list = self.init_joint_list()
        self._last_update_times = []          
        self.current_gains = []
        self._linearize = self.get_parameter('linearize_gain_scheduling').get_parameter_value().bool_value
        self._gradient = self.get_parameter('linear_slope').get_parameter_value().double_value

        self.callback_group = ReentrantCallbackGroup()
        self.create_subscription(msg_type=CurrentGait, topic='/march/gait_selection/current_gait',
                                 callback=self.gait_selection_callback, callback_group=self.callback_group,
                                 qos_profile=10)

        # Getter and setter clients to interact with the ROS1 parameter server
        self.gains_getter_client = self.create_client(GetGainValuesList, 'march/parameter_server/get_gain_values_list')
        self.gains_setter_client = self.create_client(SetGainValuesList, 'march/parameter_server/set_gain_values_list')

        self.get_current_gains_event = Event()
        self.set_current_gains_event = Event()

    def init_joint_list(self) -> List[str]:
        """
        Initialize the joint_list.

        A client the uses the ROS1 paramater server is created, called, and finally destroyed.

        :return Returns a list of joint names
        """
        joint_list_ci = self.create_client(GetParamStringList, 'march/parameter_server/get_param_string_list')
        future = joint_list_ci.call_async(GetParamStringList.Request(name='/march/joint_names'))
        rclpy.spin_until_future_complete(self, future)
        joint_list_ci.destroy()
        return future.result().value

    def gait_selection_callback(self, msg):
        self.get_logger().info('---Gait callback: ' + str(msg.gait_type) + '---')

        new_gait_type = msg.gait_type
        if new_gait_type is None or new_gait_type == '' \
                or not self.has_parameter('gait_types.{gait_type}.{joint}.p'.format(gait_type=new_gait_type,
                                                                                    joint=self._joint_list[0])):
            self.get_logger().warn('The gait has unknown gait type of `{gait_type}`, default is set to walk_like'.
                                   format(gait_type=new_gait_type))
            new_gait_type = 'walk_like'

        self._gait_type = new_gait_type
        self.get_current_gains()
        needed_gains = self.get_needed_gains()

        if not self.is_interpolation_done(needed_gains):
            self.perform_interpolation(needed_gains)

    def perform_interpolation(self, needed_gains):
        rate = self.create_rate(10)

        self.get_logger().info(f'Beginning PID interpolation for gait type: {self._gait_type}')
        begin_time = self.get_clock().now()
        self._last_update_times = len(self._joint_list) * [begin_time]
        while not self.is_interpolation_done(needed_gains):
            self.client_update(needed_gains)
            rate.sleep()
        duration = round((self.get_clock().now() - begin_time).nanoseconds / 1e6)
        self.get_logger().info(f'PID interpolation finished in {duration} ms')

    def client_update(self, needed_gains: List[List[float]]):
        """
        Perform interpolation if necessary and update the PID gains in the ROS1 parameter server.

        :param needed_gains The final gains to interpolate to
        """
        if self._linearize:
            for i in range(len(self._joint_list)):
                current_time = self.get_clock().now()
                time_interval = current_time - self._last_update_times[i]

                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains[i], self._gradient,
                                                    time_interval)

                self._last_update_times[i] = self.get_clock().now()
        else:
            self.current_gains = needed_gains

        self.update_gain_values()

    def update_gain_values(self):
        """ Update the gain values in the ROS1 parameter server.

        The gains setter client is used to make an ayschronous call.
        The set current gains event is used to wait until the future is done.
        """
        self.set_current_gains_event.clear()
        
        future = self.gains_setter_client.call_async(
            SetGainValuesList.Request(joints=self._joint_list, gain_values=GainValuesList(
                gain_values_list=[GainValues(gains=gains) for gains in self.current_gains])))
        future.add_done_callback(self.set_future_cb)

        self.set_current_gains_event.wait()

    def set_future_cb(self, future_done: Future):
        """ Callback function for when setting the gains from the ROS1 parameter server is done.
        If setting the gains was not successful, then an exception is raised.
        :param future_done: Future of the gains setter client call.
        """
        result = future_done.result()
        if not result.success:
            raise ParameterSetException(self._joint_list)
        self.set_current_gains_event.set()

    def get_current_gains(self):
        """ Get the current gains from the ROS1 parameter server.

        The gains getter client is used to make an ayschronous call.
        The get current gains event is used to wait until the future is done.
        """
        self.get_current_gains_event.clear()
        future = self.gains_getter_client.call_async(GetGainValuesList.Request(joints=self._joint_list))

        future.add_done_callback(self.get_gains_callback)
        self.get_current_gains_event.wait()

    def get_gains_callback(self, future_done: Future):
        """ Callback function for when getting the gains from the ROS1 parameter server is done.
        If getting the gains was successful, the current gains are updated, otherwise an exception is raised.
        :param future_done: Future of the gains getter client call.
        """
        result = future_done.result()
        if result.success:
            self.current_gains = [list(gain_values.gains) for gain_values in result.gain_values.gain_values_list]
        else:
            raise ParameterGetException(self._joint_list)
        self.get_current_gains_event.set()

    def get_needed_gains(self) -> List[List[int]]:
        """
        Pull the PID values from the the gains_per_gait_type.yaml config file

        :returns Returns a list, containing a list of pid gain values for each joint
        """
        gains_list = []
        for joint in self._joint_list:
            gains = self.get_parameters_by_prefix('gait_types.{gait_type}.{joint}'.
                                                  format(gait_type=self._gait_type, joint=joint))
            gains_list.append([gain.get_parameter_value().integer_value for gain in gains.values()])
        return gains_list

    def is_interpolation_done(self, needed_gains: List) -> bool:
        """
        Checks whether the current gains are equal to the needed gains for each joint

        :param needed_gains Gains that the current_gains list should be equal to if the interpolation is done
        :returns Returns True if the gains are equal, else False
        """
        return self.current_gains == needed_gains
