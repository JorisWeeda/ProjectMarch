import rclpy
from march_shared_msgs.srv import RequestParamList, RequestParamListRequest
from rclpy.node import Node


class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Hello World!')

        # _joint_list_cli = self.create_client(RequestParamList, 'march/parameter_server/request_param_list')
        # self._joint_list = _joint_list_cli.call(RequestParamListRequest('/march/joint_names')).result()
        # _joint_list_cli.destroy()
        #
        # self.get_logger().info(type(self._joint_list))
        # self.get_logger().info(str(self._joint_list))


def main(args=None):
    rclpy.init(args=args)

    example_node = ExampleNode()

    rclpy.spin(example_node)

    example_node.destroy_node()
    rclpy.shutdown()
