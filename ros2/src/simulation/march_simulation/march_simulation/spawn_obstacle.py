import os

from ament_index_python import get_package_share_directory
from rclpy.node import Node
import rclpy
from gazebo_msgs.srv import SpawnEntity
import xacro

NODE_NAME = 'obstacle_spawner'


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.obstacle = self.get_parameter('obstacle').get_parameter_value().string_value

    def spawn(self):
        while not self.spawn_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Waiting for /spawn_entity to become available')

        xacro_file = os.path.join(get_package_share_directory('march_simulation'), 'obstacles', self.obstacle) + '.xacro'

        if not os.path.exists(xacro_file):
            self.get_logger().fatal(f'Cannot find file {xacro_file}')
            return

        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)

        request = SpawnEntity.Request(name=self.obstacle,
                                      xml=doc.toxml())

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result.success:
            self.get_logger().fatal('Unable to spawn obstacle')


def main():
    rclpy.init()
    ObstacleSpawner().spawn()