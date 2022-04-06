### Purpose

This repo provides example python nodes running on ROS2. The developments were done under an Ubuntu 20.04 machine running ROS2 Foxy version. 

The demo packages showcases many of essential concep of ROS-ROS2 such as; 

* Topic Publisher/Subscriber
* Service Server/Clients
* Action Server/Clients
* Parameter loading and Launch files
* Custom message definition and usage (see `ros2_demo_custom_msgs`)

# Build

Assuming that ROS2 Foxy desktop is installed on your machine, create a workspace and build the workspace with colcon;

```bash
mkdir -p ros2_ws/src
cd ros2_ws
colcon build --symlink-install
```

You can test a sample node (e.g. the topic publisher and subscriber)

```bash
cd ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ros2_demo_python_nodes start_topic_pub_sub.launch.py
```