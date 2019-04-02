#include <ModeHeader.h>

void mode_W()
{
	geometry_msgs::Quaternion desiredQ;

    try
    {
    	//Extract desired waypoint
    	desWaypoint = stoi(desired_mode.substr(1,desired_mode.length()-1));	
    }	
    catch(const invalid_argument& err)
    {
    	ROS_ERROR("Waypoint rejected. Expected mode: W<waypoint_number>");
    	desired_mode = "0";
    	return;
    }

	//Check if waypoint number is acceptable
	if(desWaypoint <= nWaypoints && desWaypoint >= 1)
	{
		//Set desired position as waypoint
		DesiredPoseStamped.pose.position.x = xWaypoint[desWaypoint-1];
		DesiredPoseStamped.pose.position.y = yWaypoint[desWaypoint-1];
		DesiredPoseStamped.pose.position.z = zWaypoint[desWaypoint-1];

		//Set desired yaw
		desiredQ = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawWaypoint[desWaypoint-1]*M_PI/180 + M_PI/2);
		DesiredPoseStamped.pose.orientation = desiredQ;
	}
	else
	{
		//Else display error and do not set desired position
		ROS_ERROR("Waypoint rejected. Waypoint %d is not set!", desWaypoint);
	}

	//switch to default mode once we set waypoint
	desired_mode = "0";
}