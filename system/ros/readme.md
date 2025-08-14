# Recording the preparation of the orientation
Date: Aug 14
This readme is a record for what I did to try to make Sawyer work with this computer setup. 
### Background
The Sawyer robot is connected to the wifi router, and the wifi router is doing the IP assignment. The workstation PC is connected to the router wirelessly.
In order to use the Sawyer sdk, which is compatible with older version of ubuntu (18.04?) and older version of ROS, we have to set them up in a container.
The first step will be setting up the environment and dependencies correctly. 


### Container Setup
Assume we have the latest version of Docker installed, we need to create this ROS-melodic container in our machine. The reason why we use Melodic instead of Noetic is because the SDk on Sawyer is an older version (5.2.1), and it's relatively more tedious to upgrade the robot comparing to downgrading the developing environment. Thus, the 5.2.1 sdk is written in python2, so we use the Melodic which fully supports python2. 
