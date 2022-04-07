

import rclpy.action
import rclpy.node
import rclpy.qos
import rclpy
import rclpy.parameter

from ros2_demo_custom_msgs.action._navigate_to_pose import NavigateToPose
from ros2_demo_custom_msgs.action._navigate_to_pose import NavigateToPose_Goal

from geometry_msgs.msg import Twist


class ROS2ActionClient(rclpy.node.Node):

    def __init__(self, *args):

        super(ROS2ActionClient, self).__init__("ROS2ActionClient")

        use_sim_time = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([use_sim_time])

        self.action_client = rclpy.action.ActionClient(
            node=self,
            action_type=NavigateToPose,
            action_name="navigate_to_pose")

        self.action_client.wait_for_server()

        self.get_logger().info("Created the navigate_to_pose action client node")

    def test(self):

        goal = NavigateToPose_Goal()
        goal.goal_pose.header.frame_id = "odom"
        goal.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x = 26.0
        goal.goal_pose.pose.position.y = -15.0
        goal.goal_pose.pose.orientation.w = 1.0

        self.get_logger().info("sending a goal")

        self.get_logger().info("%s" % str(goal.goal_pose))

        future = self.action_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = ROS2ActionClient()

    future = node.test()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.get_logger().info("Wating for action to finish")

    res = future.result()

    node.get_logger().info("Result %s" % res.message)
    node.get_logger().info("I am dying just now , bye!")
    rclpy.spin(node=node)


if __name__ == '__main__':
    main()
