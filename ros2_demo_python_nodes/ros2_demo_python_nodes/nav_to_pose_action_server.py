from .helpers import PurePursuitController
from .helpers import SimpleNavHelpers

import rclpy.action
import rclpy.node
import rclpy.qos
import rclpy

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from geometry_msgs.msg import Twist
from ros2_demo_custom_msgs.action._navigate_to_pose import NavigateToPose

import threading


class ROS2ActionServer(rclpy.node.Node):

    def __init__(self, *args):

        super(ROS2ActionServer, self).__init__("ROS2ActionServer")
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self.action_server = ActionServer(
            node=self,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            result_timeout=2000)

        use_sim_time = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([use_sim_time])

        self.pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=rclpy.qos.qos_profile_sensor_data)

        self.helpers = SimpleNavHelpers(node=self)
        self.controller = PurePursuitController(1.0, 5.0, 0.8, 0.8)
        self.rotation_error_tolerance = 0.2
        self.dist_error_tolerance = 0.1
        self.get_logger().info("Created the navigate_to_pose server node")

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal_pose = goal_handle.request.goal_pose
        self.get_logger().info("Recieved a goal from client")
        self.get_logger().info(str(goal_pose))

        dist_to_goal_satisfied = False
        rot_to_goal_satisfied = False
        rate = self.create_rate(10)

        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        while not (dist_to_goal_satisfied and rot_to_goal_satisfied) and rclpy.ok():

            self.get_logger().info("Processing goal")

            curr_robot_pose = self.helpers.get_curr_robot_pose(
                now=self.get_clock().now(),
                logger=self.get_logger())

            curr_dist_to_goal = self.helpers.pose_euclidean_dist(
                curr_robot_pose.pose, goal_pose.pose)

            # VERY SIMPLE PURE PURSUIT CONTROLLER
            dist_error, rot_error = self.controller.compute_error(
                curr_robot_pose, goal_pose, dist_to_goal_satisfied)

            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return NavigateToPose.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPose.Result()

            if dist_error < self.dist_error_tolerance:
                self.get_logger().info(
                    "We are at goal now, adjusting to correct heading")
                dist_to_goal_satisfied = True

            if dist_to_goal_satisfied and (abs(rot_error) < self.rotation_error_tolerance):
                self.get_logger().info(
                    "Corrected the heading,")
                rot_to_goal_satisfied = True

            feedback_msg.remaining_distance_to_goal = curr_dist_to_goal
            goal_handle.publish_feedback(feedback_msg)

            if (dist_to_goal_satisfied and rot_to_goal_satisfied):
                goal_handle.succeed()
                result.success = True
                self.get_logger().info("Navigation was a success")

            v_in, w_in = self.controller.compute_velocities(
                curr_robot_pose, goal_pose, dist_to_goal_satisfied)

            # Publish required velocity commands
            computed_velocity = Twist()
            computed_velocity.linear.x = v_in
            computed_velocity.angular.z = w_in
            self.pub.publish(computed_velocity)
            rate.sleep()

        return result


def main():
    rclpy.init()
    action_server = ROS2ActionServer()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(action_server)
    rclpy.spin(action_server, executor=multi_thread_executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
