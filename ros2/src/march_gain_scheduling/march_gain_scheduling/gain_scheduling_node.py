from threading import Thread

import rclpy

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main(args=None):
    rclpy.init(args=args)

    # while not rclpy.is_shutdown() and not rclpy.has_param('/march/joint_names'):
    #     rclpy.sleep(0.5)
    #     rclpy.logdebug('Waiting on /march/joint_names to be available')
    #
    # if rclpy.is_shutdown():
    #     return

    node = DynamicPIDReconfigurer()

    rclpy.spin(node)
