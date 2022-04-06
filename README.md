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
### Additional depenedencies
Some nodes might require installation of 3rd party libraries such as gTTS (for generating speech from text). 
You should be able to install other dependencies them with `pip3` in case you get "no module XX found" errors. 

### Action and Service Nodes

Under `ros2_demo_python_nodes/ros2_demo_python_nodes/text_to_speech_service_server.py`, 
We provide two **_service_** servers to generate speech. 

For instance; 

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
A randomly picked sentence (Loaded from `params.yaml`)will be played back.

Or you can generate the speech for a specific sentence;

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 service call /speak_phrase ros2_demo_custom_msgs/srv/Phrase "{phrase: 'I will be Spoken'}"
```

The **action** nodes might come up as scary if you are a beginner, It is suggested you get grasp of ROS Actions in order to test and understand `navigate_to_pose` action interface provided in `ros2_demo_python_nodes/ros2_demo_python_nodes/text_to_speech_service_server.py`.

THe purpose of this **action** node is to drive a robot to a certain pose (hence `navigate_to_pose`), It is required that you have a valid `tf` tree,  depending on your localization setup you should have either in `map` or `odom` frame to `base_link` transforms, modify `ros2_demo_python_nodes/ros2_demo_python_nodes/helpers.py` with correct frame id (either `map` or `odom`) in the `get_curr_robot_pose` function.

To test the actions, source your workspace and do;

```bash
ros2 launch ros2_demo_python_nodes start_nav2pose_action.launch.py
```

In a seperate terminal;

```bash
ros2 run ros2_demo_python_nodes nav_to_pose_action_client
```

For this to succeed, you do need a complete `tf` transfrom from `odom`/`map` to `base_link` so that robot can know its current position and navigate itself towards the goal. You can change the goal pose in `ros2_demo_python_nodes/ros2_demo_python_nodes/nav_to_pose_action_client.py` to send robot to differnt pose.


