import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from builtin_interfaces.msg import Duration


def main():
    rclpy.init()
    ExampleJointTrajectory()

class ExampleJointTrajectory(Node):
    def __init__(self):
        super().__init__('example')

        action = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=['rotational_joint'],
                points=[
                    JointTrajectoryPoint(positions=[-2.0],
                                         time_from_start=Duration(sec=2)),
                    JointTrajectoryPoint(positions=[2.0],
                                         time_from_start=Duration(sec=4))
                ]))

        action_client = ActionClient(self, FollowJointTrajectory, '/march/follow_joint_trajectory')

        action_client.wait_for_server()
        action_client.send_goal_async(action, feedback_callback=self.fb_callback)

        rclpy.spin(self)


    def fb_callback(self, msg):
        self.get_logger().info(f'feedback: {msg.feedback}')




