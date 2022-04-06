import rclpy
import rclpy.qos
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import numpy as np
from .helpers import read_points
from .helpers import generate_points


class PCDSubPubNode(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')
        # Set up a subscription to the 'pcd' topic with a callback to the
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            'points',           # topic
            self.listener_callback,                             # Function to call
            qos_profile=rclpy.qos.qos_profile_sensor_data       # QoS
        )

        self.obstacle_pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2,
            'obstacle_pcd',
            qos_profile=rclpy.qos.qos_profile_sensor_data)

    def listener_callback(self, msg: sensor_msgs.PointCloud2):

        self.get_logger().info(str(msg.header.frame_id))

        pcd_as_numpy_array = np.array(list(read_points(msg)))[:, 0:3]

        obstacle_points = []
        free_points = []

        for curr_point in pcd_as_numpy_array:
            if curr_point[2] > -0.6:
                obstacle_points.append(curr_point)
            else:
                free_points.append(curr_point)

        obs_pcd = generate_points(
            np.array(obstacle_points), msg.header.frame_id)

        self.obstacle_pcd_publisher.publish(obs_pcd)


def main():
    rclpy.init()
    node = PCDSubPubNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
