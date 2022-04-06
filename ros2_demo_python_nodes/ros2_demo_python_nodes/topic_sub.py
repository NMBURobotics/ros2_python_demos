import rclpy
import rclpy.node
import rclpy.qos
from geometry_msgs.msg import Point


class ROS2Sub(rclpy.node.Node):
    def __init__(self, *args):
        super(ROS2Sub, self).__init__("ROS2Sub")
        self.create_subscription(
            Point, "points", self.points_callback, rclpy.qos.qos_profile_sensor_data)

    def points_callback(self, msg):
        self.get_logger().info("Recieved a point with x of %s" % msg.x)


def main():
    rclpy.init()
    node = ROS2Sub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
