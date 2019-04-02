#ifndef _MODEHEADER_H
#define _MODEHEADER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

using namespace std;

//Node handle pointer
extern ros::NodeHandle * nh;

//Extern variables needed by the modes
extern ros::Time last_request;
extern string desired_mode;
extern geometry_msgs::PoseStamped DesiredPoseStamped;
extern float current_x, current_y, current_z, start_x, start_y;
extern float takeoff_height;
extern double current_yaw;
extern mavros_msgs::State current_state;
extern bool armNow, disarmNow, flymode, publishFromGCS, followWaypoint;
extern vector<float> xWaypoint, yWaypoint, zWaypoint, yawWaypoint, tWaypoint;
extern int nWaypoints, desWaypoint;

/* --- Navigation modes --- */

//Offboard
void mode_O();
//Arm
void mode_A();
//Takeoff
void mode_T();
//Land
void mode_L();
//Disarm
void mode_DISARM();
//Home position
void mode_H();
//Record waypoint
void mode_R();
//Waypoint position
void mode_W();
//Next waypoint position
void mode_N();
//Follow waypoints
void mode_F();
//Search for objects
void mode_SEARCH();
//Clear waypoint
void mode_CW();
//Load waypoints
void mode_LW();
//Save waypoints
void mode_SW();
//Start flying
void mode_FLY();
#endif
