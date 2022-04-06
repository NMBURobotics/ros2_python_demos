from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description import LaunchDescription

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    share_dir = get_package_share_directory("ros2_demo_python_nodes")

    params = LaunchConfiguration("params")
    declare_params = DeclareLaunchArgument(name="params",
                                           default_value=share_dir+'/params.yaml',
                                           description="Full path to yaml parameters file, You will pas this to your node in order to acess to params")

    declare_node = Node(
        package="ros2_demo_python_nodes",
        executable="text_to_speech_service_server",
        name="text_to_speech_service_server",
        parameters=[params])

    ld.add_action(declare_params)
    ld.add_action(declare_node)

    return ld
