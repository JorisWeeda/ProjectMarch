from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import FollowJointTrajectoryGoal, \
    FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult, \
    FollowJointTrajectoryResult
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient


class TrajectoryScheduler(object):
    """ Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation. """
    def __init__(self, node):
        self._failed = False
        self._node = node
        self._last_goal = None

        # ROS2 solution for when gazebo ROS2 control works
        # self._action_client = ActionClient(node, FollowJointTrajectory,
        #                                    '/march_controller/follow_joint_trajectory')

        # Temporary solution to communicate with ros1 action server, should
        # be updated to use ros2 action implementation when simulation is
        # migrated to ros2
        self._trajectory_goal_pub = self._node.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic='/march/controller/trajectory/follow_joint_trajectory/goal',
            qos_profile=5)

        self._trajectory_goal_result_sub = self._node.create_subscription(
            msg_type=FollowJointTrajectoryActionResult,
            topic='march/controller/trajectory/follow_joint_trajectory/result',
            callback=self._done_cb,
            qos_profile=5
        )

    def schedule(self, trajectory):
        """Schedules a new trajectory.
        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        # while not self._action_client.wait_for_server(timeout_sec=1):
        #     self._node.get_logger().info('Waiting for follow_joint_trajectory to come online')

        self._failed = False
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._trajectory_goal_pub.publish(FollowJointTrajectoryActionGoal(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            goal_id=GoalID(), goal=goal))

        # ROS2 sending goal to gazebo
        # goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        # send_goal_future = self._action_client.send_goal_async(goal)
        # send_goal_future.add_done_callback(self._goal_response_cb)

    def failed(self):
        return self._failed

    def reset(self):
        self._failed = False

    # ROS2 Goal response
    # def _goal_response_cb(self, future):
    #     goal_handle = future.result()
    #
    #     if goal_handle.accepted:
    #         get_result_future = goal_handle.get_result_async()
    #         get_result_future.add_done_callback(self._done_cb)
    #     else:
    #         # What to do when the goal is not accepted?
    #         self._node.get_logger().error(
    #             'Unable to send FollowJointTrajectory goal')
    # def _done_cb(self, future):
    #     result = future.result().result
    #     # If not SUCCESSFUL (=0)
    #     if result.error_code != 0:
    #         self._node.get_logger().error(
    #             f'Failed to execute trajectory. {result.error_string} ({result.error_code})')
    #         self._failed = True

    def _done_cb(self, result):
        if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._node.get_logger().error(
                f'Failed to execute trajectory. {result.error_string} ({result.error_code})')
            self._failed = True

