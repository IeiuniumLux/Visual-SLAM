#include <ModeHeader.h>

void mode_R()
{
    double current_FLU_yaw;
    //Record current position as a waypoint
    xWaypoint.push_back(current_x);
    yWaypoint.push_back(current_y);
    zWaypoint.push_back(current_z);

    current_FLU_yaw = -current_yaw*180/M_PI;
    yawWaypoint.push_back(current_FLU_yaw);

    float temp_time;

    if(desired_mode.length() == 1)
    {
    	temp_time = 0.0;	
    }
    else if(desired_mode.length() == 2)
    {
        ROS_ERROR("Desired mode %s rejected. Mode usage: R-<waypoint_time>.", desired_mode.c_str());
    }
    else
    {
    	temp_time = strtof(desired_mode.substr(2,desired_mode.length()-2).c_str(),0);
    }
	tWaypoint.push_back(temp_time);

	//increment number of waypoints
   	nWaypoints++;

    //Print out recorded Waypoint info
    ROS_INFO("Waypoint W%d: %0.2f %0.2f %0.2f %0.2f %0.2f\n", nWaypoints, xWaypoint[nWaypoints-1], yWaypoint[nWaypoints-1], zWaypoint[nWaypoints-1], yawWaypoint[nWaypoints-1], tWaypoint[nWaypoints-1]);

	//Default desired mode
	desired_mode = "0";

}