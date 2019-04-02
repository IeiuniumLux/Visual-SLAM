#!/bin/bash
source /opt/ros/kinetic/setup.bash
cd ~/Visual-SLAM/gripper_ws
source devel/setup.bash

case $HOSTNAME in
	(safe50-UDOO-x86) export ROS_IP=192.168.0.222; roslaunch dynamixel_controllers controller_manager.launch;;
	(safe50-JOULE-x86) exit;;
	(*) echo "Computer not recognized!";;
esac
