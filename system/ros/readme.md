# Recording the preparation of the orientation
Date: Aug 14
This readme is a record for what I did to try to make Sawyer work with this computer setup. 
### Background
The Sawyer robot is connected to the wifi router, and the wifi router is doing the IP assignment. The workstation PC is connected to the router wirelessly.
In order to use the Sawyer sdk, which is compatible with older version of ubuntu (18.04?) and older version of ROS, we have to set them up in a container.
The first step will be setting up the environment and dependencies correctly. 


### Container Setup
Assume we have the latest version of Docker installed, we need to create this ROS-melodic container in our machine. The reason why we use Melodic instead of Noetic is because the SDk on Sawyer is an older version (5.2.1), and it's relatively more tedious to upgrade the robot comparing to downgrading the developing environment. Thus, the 5.2.1 sdk is written in python2, so we use the Melodic which fully supports python2. 

##### Container creation
```
sudo docker run -it --name sawyer_melodic \
  --net=host \
  -e ROS_MASTER_URI=http://<YOUR_PC_IP>:11311 \
  -e ROS_IP=<YOUR_PC_IP> \
  -v $HOME/ros_sawyer_ws:/root/ros_sawyer_ws \
  ros:melodic-ros-base
```
The YOUR_PC_IP will most likely change in the future, so don't forget to sub it in.

As you can read in the lines, the files will be loaded from this /ros_sawyer_ws folder under your home directory, so if anything needs to be cleaned up, that's where you go.

After exiting the container, use this to get back in `docker start -ai sawyer_melodic`

##### Container SDK setup (only first time)
```
apt update
apt install -y git build-essential python-rosdep python-vcstool python-catkin-tools ros-melodic-cv-bridge ros-melodic-vision-opencv ros-melodic-image-transport ros-melodic-control-msgs ros-melodic-actionlib-msgs ros-melodic-trajectory-msgs
rosdep init || true
rosdep update

mkdir -p /root/ros_sawyer_ws/src
cd /root/ros_sawyer_ws/src
git clone https://github.com/RethinkRobotics/intera_sdk.git
git clone https://github.com/RethinkRobotics/intera_common.git
git clone https://github.com/RethinkRobotics/sawyer_robot.git

cd /root/ros_sawyer_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="ros-melodic-joystick-drivers joystick-drivers ros-melodic-ps3joy ros-melodic-wiimote ps3joy wiimote"
catkin config --extend /opt/ros/melodic
catkin build
```
After the catkin build, you should be able to see something like `Summary: All 6 packages succeeded! ` which means, well, success. 

##### Configure `intera.sh`

