#!/bin/bash

case $HOSTNAME in
	(safe50-UDOO-x86) export ROS_IP=192.168.0.222;;
	(safe50-JOULE-x86) export ROS_IP=192.168.0.111;;
	(*) echo "Computer not recognized!";;
esac

rostopic pub /navigation_mode std_msgs/String $1
