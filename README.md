### Purpose

This repo provides example python nodes running on ROS2. The developments were done under an Ubuntu 20.04 machine running ROS2 Foxy version. 

The demo packages showcases many of essential concept in ROS-ROS2 such as; 

* Topic Publisher/Subscriber
* Service Server/Clients
* Action Server/Clients
* Parameter loading and Launch files
* Custom message definition and usage (see `ros2_demo_custom_msgs`)

### Build

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

### Action and Service Nodes

Under `ros2_demo_python_nodes/ros2_demo_python_nodes/text_to_speech_service_server.py`, We provide two service servers where a service call triggers server to generate speech. 

Fo instance; 

In a terminal; 
```bash
cd ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ros2_demo_python_nodes start_speech_server.launch.py
```
In a seperate terminal

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 service call /speak_random std_srvs/srv/Trigger
```
A randomly picked sentence (Loaded from `params.yaml`)with be played back

Or you can generate the speech for a specific sentence;
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 service call /speak_phrase ros2_demo_custom_msgs/srv/Phrase "{phrase: 'I will be Spoken'}"
```

The action nodes could come upo as scary if you are a beginner, It is suggested you get grasp of ROS Actions in order to test and understand `navigate_to_pose` action interface provided in `ros2_demo_python_nodes/ros2_demo_python_nodes/text_to_speech_service_server.py`.

THe purpose of this action is to drive a robot to a certain pose (hence `navigate_to_pose`)