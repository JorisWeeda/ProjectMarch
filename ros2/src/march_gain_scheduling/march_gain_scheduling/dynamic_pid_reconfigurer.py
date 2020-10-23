from threading import Event
from typing import List

from march_shared_classes.exceptions.parameter_exceptions import ParameterGetException, \
    ParameterSetException

from march_shared_msgs.msg import CurrentGait, GainValues, GainValuesList
from march_shared_msgs.srv import GetGainValuesList, GetParamStringList, SetGainValuesList

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from .one_step_linear_interpolation import interpolate

GAINS = ['p', 'i', 'd']


class DynamicPIDReconfigurer(Node):
    """The DynamicPIDReconfigur class is a node that performs gain scheduling.

    This means that, based on in which category a gait falls, the PID values are tuned during runtime.
    The PID values are loaded from the configuration files in the config/ folder.
    Every 10ms, the node checks whether interpolation is necessary. If it is, interpolation is done and the 
    SetGainValuesList service of the ROS1 parameter server is used to update the PID values.
    """

    def __init__(self):
        """Initialize the DynamicPIDReconfigurer / gain_scheduling node."""
        super().__init__('gain_scheduling_node', automatically_declare_parameters_from_overrides=True)
        self._gait_type = None
        self._joint_list = self.init_joint_list()
        self._last_update_time = None
        self._interpolation_running = False

        self.current_gains = np.zeros((len(self._joint_list), len(GAINS)))
        self.needed_gains = np.zeros((len(self._joint_list), len(GAINS)))

        self._linearize = self.get_parameter('linearize_gain_scheduling').get_parameter_value().bool_value
        self._gradient = self.get_parameter('linear_slope').get_parameter_value().double_value

        self.create_subscription(msg_type=CurrentGait, topic='/march/gait_selection/current_gait',
                                 callback=self.gait_selection_callback, callback_group=ReentrantCallbackGroup(),
                                 qos_profile=10)

        # Getter and setter clients to interact with the ROS1 parameter server
        self._gains_getter_client = self.create_client(GetGainValuesList, 'march/parameter_server/get_gain_values_list')
        self._gains_setter_client = self.create_client(SetGainValuesList, 'march/parameter_server/set_gain_values_list')

        self._get_current_gains_event = Event()
        self._set_current_gains_event = Event()

    def init_joint_list(self) -> List[str]:
        """Initialize the joint_list.

        A client the uses the ROS1 paramater server is created, called, and finally destroyed.

        :return Returns a list of joint names
        """
        joint_list_ci = self.create_client(GetParamStringList, 'march/parameter_server/get_param_string_list')
        joint_list_ci.wait_for_service()
        future = joint_list_ci.call_async(GetParamStringList.Request(name='/march/joint_names'))
        rclpy.spin_until_future_complete(self, future)
        joint_list_ci.destroy()
        return future.result().value

    def gait_selection_callback(self, msg: CurrentGait):
        """Callback for when a new gait is selected.

        This callback checks whether the type of the gait is different, and updates the needed gains if necessary.
        If the new gait type is an invalid type, the gait type default to 'walk_like'.
        """
        new_gait_type = msg.gait_type
        if new_gait_type is None or new_gait_type == '' \
                or not self.has_parameter('gait_types.{gait_type}.{joint}.{gain}'.format(gait_type=new_gait_type,
                                                                                         joint=self._joint_list[0],
                                                                                         gain=GAINS[0])):
            self.get_logger().warn('The gait has unknown gait type of `{gait_type}`, default is set to walk_like'.
                                   format(gait_type=new_gait_type))
            new_gait_type = 'walk_like'

        if new_gait_type != self._gait_type:
            self._gait_type = new_gait_type
            self.get_needed_gains()
            self._last_update_time = self.get_clock().now()

            self.get_logger().info(f"Updated gain scheduling configuration to '{new_gait_type}'")
            if not self._interpolation_running:
                self.start_interpolation()

    def start_interpolation(self):
        """Start the interpolation cycle.
        
        Every 100 ms (1/10 s), the current gains are retrieved. These current gains are then used to check whether
        interpolation is necessary and if it is necessary they are used for interpolation.
        """
        self._interpolation_running = True

        rate = self.create_rate(10)
        while rclpy.ok():
            self.get_current_gains()
            if not self.is_interpolation_done():
                self.client_update()
            rate.sleep()

    def client_update(self):
        """Performs interpolation if necessary and update the PID gains in the ROS1 parameter server."""
        if self._linearize:
            # Convert time_interval to seconds
            delta_t = (self.get_clock().now() - self._last_update_time).nanoseconds / 1e9
            self.current_gains = interpolate(self.current_gains, self.needed_gains, self._gradient, delta_t)
            self._last_update_time = self.get_clock().now()
        else:
            self.current_gains = self.needed_gains
        self.update_gain_values()

    def update_gain_values(self):
        """Update the gain values in the ROS1 parameter server.

        The gains setter client is used to make an ayschronous call.
        The set current gains event is used to wait until the future is done.
        """
        self._set_current_gains_event.clear()
        
        future = self._gains_setter_client.call_async(
            SetGainValuesList.Request(joints=self._joint_list, gain_values_list=GainValuesList(
                gain_values_list=[GainValues(gains=list(gains)) for gains in self.current_gains])))
        future.add_done_callback(self.set_future_cb)

        self._set_current_gains_event.wait()

    def set_future_cb(self, future_done: Future):
        """Callback function for when setting the gains from the ROS1 parameter server is done.

        If setting the gains was not successful, then an exception is raised.

        :param future_done: Future of the gains setter client call.
        """
        result = future_done.result()
        if not result.success:
            raise ParameterSetException(self._joint_list)
        self._set_current_gains_event.set()

    def get_current_gains(self):
        """Get the current gains from the ROS1 parameter server.

        The gains getter client is used to make an ayschronous call.
        The get current gains event is used to wait until the future is done.
        """
        self._get_current_gains_event.clear()
        future = self._gains_getter_client.call_async(GetGainValuesList.Request(joints=self._joint_list))

        future.add_done_callback(self.get_gains_callback)
        self._get_current_gains_event.wait()

    def get_gains_callback(self, future_done: Future):
        """Callback function for when getting the gains from the ROS1 parameter server is done.

        If getting the gains was successful, the current gains are updated, otherwise an exception is raised.

        :param future_done: Future of the gains getter client call.
        """
        result = future_done.result()
        if result.success:
            self.current_gains = np.array([gain_values.gains
                                           for gain_values in result.gain_values_list.gain_values_list])
        else:
            raise ParameterGetException(self._joint_list)
        self._get_current_gains_event.set()

    def get_needed_gains(self):
        """Pull the PID values from the the gains_per_gait_type.yaml config file

        :returns Returns a list, containing a list of pid gain values for each joint
        """
        gains_list = np.zeros((len(self._joint_list), len(GAINS)), dtype=float)
        for index, joint in enumerate(self._joint_list):
            gains = self.get_parameters_by_prefix('gait_types.{gait_type}.{joint}'.
                                                  format(gait_type=self._gait_type, joint=joint))
            gains_list[index] = [gain.get_parameter_value().integer_value for gain in gains.values()]
        self.needed_gains = gains_list

    def is_interpolation_done(self) -> bool:
        """Checks whether the current gains are equal to the needed gains for each joint

        :returns Returns True if the gains are equal, else False
        """
        return (self.current_gains == self.needed_gains).all()
