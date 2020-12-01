from typing import List
from threading import Event
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from march_shared_msgs.srv import SetObstacleSize
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
import xacro

from march_simulation.util import get_model_list

# All obstacles that have a macro file which allows for changing the dimension
# this means there must be an <name>_macro.xacro in the obstacles directory
# which accepts the parameters length, width and height with default values


RESIZABLE_OBSTACLES = ['bench']

NODE_NAME = 'set_obstacle_node'

class ObstacleDimensionSetter(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_model')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

        self.get_model_list_event = Event()
        self.spawn_entity_event = Event()

        self.create_service(SetObstacleSize, '/march/set_obstacle_size', self.set_size_callback, callback_group=ReentrantCallbackGroup())

        self.models = []

    def set_size_callback(self, request, response):
        if request.obstacle_name not in RESIZABLE_OBSTACLES:
            response.success = False
        else:
            self.set_size(name=request.obstacle_name, length=request.new_length,
                     width=request.new_width, height=request.new_height)
            response.success = True
        return response

    def set_size(self, name, length=0, width=0, height=0):
        length = 'length="{length}"'.format(
            length=length) if length != 0 else ''
        width = 'width="{width}"'.format(width=width) if width != 0 else ''
        height = 'height="{height}"'.format(
            height=height) if height != 0 else ''

        doc = xacro.parse('''<?xml version="1.0"?>
                <robot name="{name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <xacro:include filename="$(find march_simulation)/obstacles/{name}_macro.xacro"/>
                    <xacro:{name} {length} {width} {height}/>
                </robot>
                '''.format(name=name, length=length, width=width,
                           height=height), None)
        xacro.process_doc(doc)
        new_obstacle = doc.toprettyxml(indent='  ')

        self.get_model_list_event.clear()
        future = self.get_model_list_client.call_async(GetModelList.Request())
        future.add_done_callback(self.get_model_list_cb)
        self.get_model_list_event.wait()

        if name in self.models:
            self.delete_entity_client.call(DeleteEntity.Request(model_name=name))

        self.spawn_entity_event.clear()
        future = self.spawn_entity_client.call_async(SpawnEntity.Request(name=name, xml=new_obstacle))
        future.add_done_callback(self.spawn_entity_cb)
        self.spawn_entity_event.wait()

    def spawn_entity_cb(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().fatal('Unable to spawn obstacle')
        self.spawn_entity_event.set()

    def get_model_list_cb(self, future):
        result = future.result()
        if result.success:
            self.models = result.model_names
        self.get_model_list_event.set()


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = ObstacleDimensionSetter()
    rclpy.spin(node, executor)

