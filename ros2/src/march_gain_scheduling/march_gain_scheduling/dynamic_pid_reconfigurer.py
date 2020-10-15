# from dynamic_reconfigure.client import Client
# import rospy
from threading import Thread
from typing import List

import rclpy

from march_shared_msgs.srv import GetParamStringList, GetParamFloat, SetParamFloat
from march_shared_msgs.msg import CurrentGait
from rclpy.executors import Executor, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer(Node):
    def __init__(self):
        super().__init__('gain_scheduling_node', automatically_declare_parameters_from_overrides=True)

        self._gait_type = None

        # Get joint list from ROS1 parameter server
        joint_list_ci = self.create_client(GetParamStringList, 'march/parameter_server/get_param_string_list')
        future = joint_list_ci.call_async(GetParamStringList.Request(name='/march/joint_names'))
        rclpy.spin_until_future_complete(self, future)
        self._joint_list = future.result().value
        joint_list_ci.destroy()

        self._last_update_times = []
        # self.current_gains =[]
        self.current_gains = {joint: {gain: None for gain in ['p', 'i', 'd']} for joint in self._joint_list}

        self.create_subscription(msg_type=CurrentGait, topic='/march/gait_selection/current_gait',
                                 callback=self.gait_selection_callback, qos_profile=10)

        self._linearize = self.get_parameter('linearize_gain_scheduling').get_parameter_value().bool_value
        self._gradient = self.get_parameter('linear_slope').get_parameter_value().double_value

        # Getter and setter clients to interact with the ROS1 parameter server
        self.float_getter_client = self.create_client(GetParamFloat, 'march/parameter_server/get_param_float')
        self.float_setter_client = self.create_client(SetParamFloat, 'march/parameter_server/set_param_float')

        # # For debug
        # self.load_current_gains()
        # needed_gains = [[12.34, 12.34, 12.34], [12.34, 12.34, 12.34], [12.34, 12.34, 12.34], [12.34, 12.34, 12.34],
        #                 [12.34, 12.34, 12.34], [12.34, 12.34, 12.34], [12.34, 12.34, 12.34], [12.34, 12.34, 12.34]]
        # self.client_update(needed_gains)

    def gait_selection_callback(self, msg):
        self.get_logger().info('Gait callback: ' + str(msg))

        new_gait_type = msg.gait_type
        if new_gait_type is None or new_gait_type == '' \
                or not self.has_parameter('gait_types.{gait_type}.{joint}.p'.format(gait_type=new_gait_type,
                                                                                    joint=self._joint_list[0])):
            self.get_logger().warn('The gait has unknown gait type of `{gait_type}`, default is set to walk_like'.
                                   format(gait_type=new_gait_type))
            new_gait_type = 'walk_like'

        self._gait_type = new_gait_type
        self.load_current_gains()
        needed_gains = [self.look_up_table(i) for i in range(len(self._joint_list))]

        # if not self.is_interpolation_done(needed_gains):
        #     rate = self.create_rate(10)
        #
        #     self.get_logger().info('Beginning PID interpolation for gait type: {0}'.format(self._gait_type))
        #     begin_time = self.get_clock().now()
        #     self._last_update_times = len(self._joint_list) * [begin_time]
        #     while not self.is_interpolation_done(needed_gains):
        #         self.client_update(needed_gains)
        #         rate.sleep()
        #     self.get_logger().info('PID interpolation finished in {0}s'.format(self.get_clock().now() - begin_time))

    def client_update(self, needed_gains: List[List[float]]):
        """
        Update the PID gains in the ROS1 parameter server.

        Interpolates if _linearize is true.
        Then updates the current_gains using the update_gain_values() function.

        :param needed_gains The final gains to interpolate to
        """
        for i in range(len(self._joint_list)):
            if self._linearize:
                current_time = self.get_clock().now()
                time_interval = current_time - self._last_update_times[i]

                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains[i], self._gradient,
                                                    time_interval)

                self._last_update_times[i] = self.get_clock().now()
            else:
                self.current_gains[i] = needed_gains[i]
            self.update_gain_values(self._joint_list[i], self.current_gains[i])

    def update_gain_values(self, joint: str, gains: List[float]):
        """
        Update the gain values for a certain joint

        :param joint Joint to update gain values for
        :param gains Gain values to set in the ROS1 parameter server
        """
        for gain, value in zip(['p', 'i', 'd'], gains):
            future = self.float_setter_client.call_async(
                SetParamFloat.Request(name='/march/controller/trajectory/gains/{joint}/{gain}'.
                                      format(joint=joint, gain=gain), value=value))
            rclpy.spin_until_future_complete(self, future)

            if value != future.result().value:
                raise RuntimeError('Setting pid value error')

    def load_current_gains(self):
        """
        Load the current gains from the ROS1 parameter server, and put them in self.current_gains.
        Calls the 'march/parameter_server/get_param_float' 24 times (8 joints * 3 pid values)
        """
        # self.current_gains = []
        self.current_gains = {joint: {gain: None for gain in ['p', 'i', 'd']} for joint in self._joint_list}
        for joint_name in self._joint_list:
            # gains =[]
            for gain in ['p', 'i', 'd']:
                future = self.float_getter_client.call_async(GetParamFloat.Request(
                    name='/march/controller/trajectory/gains/{joint}/{gain}'.format(joint=joint_name, gain=gain)))

                # rclpy.spin_until_future_complete(self, future)
                future.add_done_callback(lambda future_done: self.future_cb(future_done, joint_name, gain))


            # self.get_logger().info(str(gains))
            # self.current_gains.append(gains)

    def future_cb(self, future_done, joint, gain):
        self.get_logger().info(str(future_done))
        self.current_gains[joint][gain] = future_done.result().value

    def look_up_table(self, joint_index: int) -> List[int]:
        """
        Pulls the PID values from the the gains_per_gait_type.yaml config file

        :param joint_index Index of the joint to lookup
        :returns Returns a list containg the pid values for that joint
        """
        gains = self.get_parameters_by_prefix('gait_types.{gait_type}.{joint}'.
                                              format(gait_type=self._gait_type, joint=self._joint_list[joint_index]))
        return [gain.get_parameter_value().integer_value for gain in gains.values()]

    def is_interpolation_done(self, needed_gains: List) -> bool:
        """
        Checks whether the current gains are equal to the needed gains for each joint

        :param needed_gains Gains that the current_gains list should be equal to if the interpolation is done
        :returns Returns True if the gains are equal, else False
        """
        done = True
        for i in range(len(self._joint_list)):
            if self.current_gains[i] != needed_gains[i]:
                done = False
        self.get_logger().info('Interpolation done: ' + str(done))
        return done
