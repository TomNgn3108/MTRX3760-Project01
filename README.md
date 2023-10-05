# MTRX3760 - Major Project 1

### Group Members
* Gavin Gao
* Zheng Li
* Thomas Nguyen
* Siddarth Ramasubramanian

### GIT pw
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

### Instructions

Launching Gazebo
- cd catkin_ws
- catkin_make
- source ./devel/setup.bash
- export TURTLEBOT3_MODEL=burger
- roslaunch turtlebot3_gazebo turtlebot3_world.launch

Launching Teleop
- cd catkin_ws
- catkin_make
- source ./devel/setup.bash
- export TURTLEBOT3_MODEL=burger
- roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Launching Automation in gazebo
- cd catkin_ws
- catkin_make
- source ./devel/setup.bash
- export TURTLEBOT3_MODEL=burger
- roslaunch turtlebot3_gazebo <.launch file_name>
