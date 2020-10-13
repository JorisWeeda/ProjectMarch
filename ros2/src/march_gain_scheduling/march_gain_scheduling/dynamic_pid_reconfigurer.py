# from dynamic_reconfigure.client import Client
# import rospy
import rclpy

from march_shared_msgs.srv import RequestParamList, RequestParamFloat, SetParamFloat
from march_shared_msgs.msg import CurrentGait

from rclpy.node import Node
from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer(Node):
    def __init__(self):
        super().__init__('gain_scheduling_node')

        self._gait_type = None

        self._joint_list = get_param_async('list', '/march/joint_names', self)

        self._last_update_times = []
        # self._clients = [Client('/march/controller/trajectory/gains/' + joint, timeout=90) for joint in
        #                  self._joint_list]
        self.current_gains = []

        # self.create_subscription(msg_type=CurrentGait, topic='/march/gait_selection/current_gait',
        #                          callback=self.gait_selection_callback, qos_profile=10)

        self._linearize = self.get_parameter('linearize_gain_scheduling')
        self._gradient = self.get_parameter('linear_slope')

    def gait_selection_callback(self, msg):
        new_gait_type = msg.gait_type
        if new_gait_type is None or new_gait_type == '' \
                or not self.has_parameter('gait_types/{gait_type}'.format(gait_type=new_gait_type)):
            self.get_logger().warn('The gait has unknown gait type of `{gait_type}`, default is set to walk_like'.format(
                gait_type=new_gait_type))
            new_gait_type = 'walk_like'

        self._gait_type = new_gait_type
        self.load_current_gains()
        needed_gains = [self.look_up_table(i) for i in range(len(self._joint_list))]

        if not self.done_interpolation_test(needed_gains):
            rate = rospy.Rate(10)

            self.get_logger().info('Beginning PID interpolation for gait type: {0}'.format(self._gait_type))
            begin_time = rospy.get_time()
            self._last_update_times = len(self._joint_list) * [begin_time]
            while not self.done_interpolation_test(needed_gains):
                self.client_update(needed_gains)
                rate.sleep()
            self.get_logger().info('PID interpolation finished in {0}s'.format(rospy.get_time() - begin_time))

    def client_update(self, needed_gains):
        for i in range(len(self._joint_list)):
            if self._linearize:
                current_time = rospy.get_time()
                time_interval = current_time - self._last_update_times[i]

                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains[i], self._gradient,
                                                    time_interval)

                self._last_update_times[i] = rospy.get_time()
            else:
                self.current_gains[i] = needed_gains[i]

            self._clients[i].update_configuration({'p': self.current_gains[i][0],
                                                   'i': self.current_gains[i][1],
                                                   'd': self.current_gains[i][2]})

    def load_current_gains(self):
        self.current_gains = []
        for joint_name in self._joint_list:
            p_gain = get_param_async('float', '/march/controller/trajectory/gains/' + joint_name + '/p', self)
            i_gain = get_param_async('float', '/march/controller/trajectory/gains/' + joint_name + '/i', self)
            d_gain = get_param_async('float', '/march/controller/trajectory/gains/' + joint_name + '/d', self)
            self.current_gains.append([p_gain, i_gain, d_gain])

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, joint_index):
        gains = self.get_parameter('gait_types/' + self._gait_type + '/' + self._joint_list[joint_index]).\
            get_parameter_value()
        # gains = rospy.get_param('~gait_types/' + self._gait_type + '/' + self._joint_list[joint_index],
        #                         {'p': None, 'i': None, 'd': None})
        return [gains['p'], gains['i'], gains['d']]

    def done_interpolation_test(self, needed_gains):
        done = True
        for i in range(len(self._joint_list)):
            if self.current_gains[i] != needed_gains[i]:
                done = False
        return done


def get_param_async(type: str, name: str, node: Node):
    request_types = {'list': RequestParamList, 'float': RequestParamFloat}

    client = node.create_client(request_types[type], 'march/parameter_server/request_param_' + type)
    future = client.call_async(request_types[type].Request(name=name))

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            value = future.result().value
            client.destroy()
            return value


def set_param_async(type: str, name: str, node: Node):
    set_types = {'float': SetParamFloat}

    client = node.create_client(set_types[type], 'march/parameter_server/set_param_' + type)
    client.call(set_types[type].Request(name=name))
