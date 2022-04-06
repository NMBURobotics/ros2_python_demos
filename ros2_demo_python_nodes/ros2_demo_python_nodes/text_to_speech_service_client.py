from cgi import test

from pytest import ExitCode
import rclpy.node
import rclpy.qos
import rclpy
import rclpy.executors
import rclpy.exceptions

from ros2_demo_custom_msgs.srv._phrase import Phrase, Phrase_Request, Phrase_Response
from std_srvs.srv._trigger import Trigger, Trigger_Request, Trigger_Response
from threading import Thread


class ROS2ServiceClientNode(rclpy.node.Node):
    def __init__(self, *args):
        super(ROS2ServiceClientNode, self).__init__("client_node")

        self.random_client = self.create_client(
            Trigger, "speak_random", qos_profile=rclpy.qos.qos_profile_services_default)
        self.speak_phrase = self.create_client(
            Phrase, "speak_phrase", qos_profile=rclpy.qos.qos_profile_services_default)

        self.random_client.wait_for_service()
        self.speak_phrase.wait_for_service()
        self.get_logger().info("Connected to all services")

    # NOT RECCOMENDED WAY OF CALLING SERVICES
    def test_services_sync(self):
        random_phrase_req = Trigger_Request()
        try:
            # Blocking call
            res = self.random_client.call(random_phrase_req)
            self.get_logger().info("Called random_client")
            self.get_logger().info("Result %s" % res.message)
        except rclpy.exceptions.InvalidServiceNameException:

            self.get_logger().error("Failed to call random_client")
    # RECCOMENDED WAY OF CALLING SERVICES

    def test_services_async(self):
        speak_phrase_req = Phrase_Request()
        speak_phrase_req.phrase = "Hello, I am a really really really really really loooooong sentence "
        try:
            # NON Blocking call
            future = self.speak_phrase.call_async(speak_phrase_req)
        except rclpy.exceptions.InvalidServiceNameException:
            self.get_logger().error("Failed to call speak_phrase")
            return ExitCode
        return future


def main():

    rclpy.init()
    node = ROS2ServiceClientNode()

    #  BLOCKING Service SPECIFIC CODE
    # spin_thread = Thread(target=rclpy.spin, args=(node,))
    # spin_thread.start()
    # node.test_services_sync()

    # NON-BLOCKING Service SPECIFIC CODE
    future = node.test_services_async()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.get_logger().info("Wating for service to finish")
    res = future.result()
    node.get_logger().info("Result %s" % res.message)
    node.get_logger().info("I am dying just now , bye!")

    # SHUT DOWN
    rclpy.shutdown()


if __name__ == '__main__':
    main()
