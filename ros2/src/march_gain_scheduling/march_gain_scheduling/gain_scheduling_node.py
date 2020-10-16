import rclpy
from rclpy.executors import MultiThreadedExecutor

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    node = DynamicPIDReconfigurer()

    rclpy.spin(node, executor)
