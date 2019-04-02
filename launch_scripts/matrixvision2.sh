#!/bin/bash
source /opt/ros/kinetic/setup.bash
cd ~/Visual-SLAM/mvbluefox_ws
source devel/setup.bash

case $HOSTNAME in
	(safe50-UDOO-x86) export ROS_IP=192.168.0.222; roslaunch bluefox2 bluefox2.launch device:=25001964;;
	(safe50-JOULE-x86) exit;;
	(*) echo "Computer not recognized!";;
esac
