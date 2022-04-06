from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description import LaunchDescription


def generate_launch_description():

    ld = LaunchDescription()

    node = Node(package="ros2_demo_python_nodes",
                executable="pointcloud_sub_pub",
                name="pointcloud_sub_pub",
                remappings=[('points', 'ouster/points'),
                            ('obstacle_pcd', 'obstacle_pcd')])

    ld.add_action(node)

    return ld
