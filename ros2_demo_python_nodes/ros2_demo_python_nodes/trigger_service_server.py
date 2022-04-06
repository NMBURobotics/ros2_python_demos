import rclpy
import rclpy.node
import rclpy.qos

from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Response
from std_srvs.srv._trigger import Trigger_Request
import time


class ROS2TriggerServiceServer(rclpy.node.Node):

    def __init__(self, *args):
        super(ROS2TriggerServiceServer, self).__init__(
            "ROS2TriggerServiceServer")
        self.create_service(srv_type=Trigger,
                            srv_name="trigger_service",
                            callback=self.service_server_callback,
                            qos_profile=rclpy.qos.qos_profile_services_default)
        self.get_logger().info("Created the server node")

    def service_server_callback(self, req, res):
        self.get_logger().info("Recieved a reqiest to trigger_service ")
        res = Trigger_Response()
        self.get_logger().info("Started to Execute your order")
        # simulate a long running task to observe async behviour of ros2 services
        time.sleep(5)
        self.get_logger().info("Finished to Execute your order")
        res.message = "Finished your order"
        res.success = True
        return res


def main():
    rclpy.init()
    node = ROS2TriggerServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
