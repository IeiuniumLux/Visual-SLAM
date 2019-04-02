#!/bin/bash
source /opt/ros/kinetic/setup.bash

case $HOSTNAME in
	(safe50-UDOO-x86) source /home/$USER/Visual-SLAM/teraranger_ws/devel/setup.bash; rosrun terarangerone terarangerone_node;;
	(safe50-JOULE-x86) source /home/$USER/Visual-SLAM/rangefinder_ws/devel/setup.bash; roslaunch altitude_sensor altitude_node.launch;;
	(*) echo "Computer not recognized!";;
esac
