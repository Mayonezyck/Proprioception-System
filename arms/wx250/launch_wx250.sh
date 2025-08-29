#!/bin/bash

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source Interbotix workspace
source ~/interbotix_ws/install/setup.bash

# Launch the robot control
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

