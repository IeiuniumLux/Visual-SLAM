#!/bin/bash

source /opt/ros/kinetic/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Visual-SLAM/Examples/ROS
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/roscore.sh'
sleep 10
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/mavros_start.sh'
sleep 5
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/rangefinder.sh'
sleep 5
#gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/realsense.sh'
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/matrixvision.sh'
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/matrixvision2.sh'
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/gripper.sh'
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/SLAM.sh'
sleep 30
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/rostopic_current.sh'
gnome-terminal -x '/home/'$USER'/Visual-SLAM/launch_scripts/rostopic_desired.sh'
