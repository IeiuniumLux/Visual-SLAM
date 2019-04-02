#!/bin/bash
source /opt/ros/kinetic/setup.bash

case $HOSTNAME in
	(safe50-UDOO-x86) export ROS_IP=192.168.0.222;;
	(safe50-JOULE-x86) export ROS_IP=192.168.0.111;;
	(*) echo "Computer not recognized!";;
esac

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Visual-SLAM/Examples/ROS

cd ~/Visual-SLAM
roslaunch Visual-SLAM.launch
