#! /bin/bash

source ~/X14-Core/ros/devel/setup.bash
export ROS_IP=192.168.1.3
export ROS_PYTHON_VERSION=3
roslaunch ~/X14-Core/ros/launch/run_rov.launch
