import rclpy
import rclpy.node
import rclpy.qos

from geometry_msgs.msg import Point


class ROS2Pub(rclpy.node.Node):

    def __init__(self, *args):
        super(ROS2Pub, self).__init__("topic_pub")
        self.my_pub = self.create_publisher(
            Point, "points", rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Started the node")

    def timer_callback(self):
        msg = Point()
        msg.x = 10.0
        self.my_pub.publish(msg)
        self.get_logger().info("Published a point with x of %s" % msg.x)


def main():
    rclpy.init()
    node = ROS2Pub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
