from setuptools import setup
from glob import glob
import os

package_name = 'ros2_demo_python_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atas',
    maintainer_email='fetulahatas1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Actual ROS nodes
    entry_points={
        'console_scripts': [
            'topic_pub = ros2_demo_python_nodes.topic_pub:main',
            'topic_sub = ros2_demo_python_nodes.topic_sub:main',
            'text_to_speech_service_server = ros2_demo_python_nodes.text_to_speech_service_server:main',
            'text_to_speech_service_client = ros2_demo_python_nodes.text_to_speech_service_client:main',
            'trigger_service_server = ros2_demo_python_nodes.trigger_service_server:main',
            'trigger_service_client = ros2_demo_python_nodes.trigger_service_client:main',
            'nav_to_pose_action_server = ros2_demo_python_nodes.nav_to_pose_action_server:main',
            'nav_to_pose_action_client = ros2_demo_python_nodes.nav_to_pose_action_client:main',
            'pointcloud_sub_pub = ros2_demo_python_nodes.pointcloud_sub_pub:main'
        ],
    },
)
