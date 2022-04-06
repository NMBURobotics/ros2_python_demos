import rclpy
import rclpy.node
import rclpy.qos

from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response
from std_srvs.srv._trigger import Trigger_Request
import time


class ROS2TriggerServiceClient(rclpy.node.Node):

    def __init__(self, *args):
        super(ROS2TriggerServiceClient, self).__init__(
            "ROS2TriggerServiceClient")
        self.client = self.create_client(srv_type=Trigger, srv_name="trigger_service",
                                         qos_profile=rclpy.qos.qos_profile_services_default)
        self.client.wait_for_service()
        self.get_logger().info("Created the client node and connected to trigger_service")

    def test_server_callback(self):
        req = Trigger_Request()
        self.client.call_async(req)


def main():
    rclpy.init()
    node = ROS2TriggerServiceClient()
    node.test_server_callback()
    node.get_logger().info("I am dying")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
