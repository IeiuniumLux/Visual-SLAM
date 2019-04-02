#include <ModeHeader.h>

void mode_CW()
{
	int clearWaypoint;
	try
    {
    	clearWaypoint = stoi(desired_mode.substr(2,desired_mode.length()-1));
    	if(clearWaypoint>=1 && clearWaypoint<=nWaypoints)
    	{
    		xWaypoint.erase (xWaypoint.begin() + clearWaypoint - 1);
    		yWaypoint.erase (yWaypoint.begin() + clearWaypoint - 1);
    		zWaypoint.erase (zWaypoint.begin() + clearWaypoint - 1);
            yawWaypoint.erase (yawWaypoint.begin() + clearWaypoint - 1);
    		tWaypoint.erase (tWaypoint.begin() + clearWaypoint - 1);    				
    		nWaypoints = nWaypoints - 1;
    		ROS_WARN("Order of waypoints has been changed!");
    	}
    	else
    	{
    		ROS_ERROR("Clear rejected. Waypoint %d is not set!", clearWaypoint);
    	}
    }
    catch(const invalid_argument& err)
    {
    	if(desired_mode.compare(2,1,"A")==0)
    	{
    		xWaypoint.erase (xWaypoint.begin(), xWaypoint.begin() + nWaypoints);
    		yWaypoint.erase (yWaypoint.begin(), yWaypoint.begin() + nWaypoints);
    		zWaypoint.erase (zWaypoint.begin(), zWaypoint.begin() + nWaypoints);
            yawWaypoint.erase (yawWaypoint.begin(), yawWaypoint.begin() + nWaypoints);            
    		tWaypoint.erase (tWaypoint.begin(), tWaypoint.begin() + nWaypoints);    				
    		nWaypoints = 0;
    		ROS_WARN("All waypoints erased!");
    	}
    }

    //change back to default mode
    desired_mode = "0";
	
}