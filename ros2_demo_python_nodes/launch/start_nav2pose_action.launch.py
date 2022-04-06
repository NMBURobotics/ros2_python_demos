from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description import LaunchDescription

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    declare_node = Node(
        package="ros2_demo_python_nodes",
        executable="nav_to_pose_action_server",
        name="nav_to_pose_action_server",
        remappings=[('cmd_vel', 'vox_nav/cmd_vel')])  # 'vox_nav/cmd_vel' to the topic that your robot listens

    ld.add_action(declare_node)

    return ld
