#include <ModeHeader.h>

void mode_N()
{
    if(desWaypoint < nWaypoints)
    {
    	desired_mode = "W" + to_string(desWaypoint+1);	
    }
    else
    {
    	ROS_ERROR("No more waypoints. Landing!");
    	desired_mode = "L";
    }
}