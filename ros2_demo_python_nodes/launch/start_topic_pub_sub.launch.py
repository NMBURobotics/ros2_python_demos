from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description import LaunchDescription


def generate_launch_description():

    ld = LaunchDescription()

    pub_node = Node(package="ros2_demo_python_nodes",
                    executable="topic_pub",
                    name="topic_pub",
                    remappings=[('points', 'my/points')])

    sub_node = Node(package="ros2_demo_python_nodes",
                    executable="topic_sub",
                    name="topic_sub",
                    remappings=[('points', 'my/points')])

    ld.add_action(pub_node)
    ld.add_action(sub_node)

    return ld
