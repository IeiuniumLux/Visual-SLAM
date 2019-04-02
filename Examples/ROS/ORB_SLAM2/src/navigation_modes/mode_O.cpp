#include <ModeHeader.h>

void mode_O()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	ros::ServiceClient set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//change mode only if current mode is not Offboard mode, and wait 5 seconds since previous request	
    if(current_state.mode!="OFFBOARD" && (ros::Time::now()-last_request > ros::Duration(5.0)))
    {
    	//call mavros service to change to offboard and check if it was successful
	    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
        {
        	//display offboard mode and change it to default 0
			ROS_INFO("OFFBOARD enabled");
           	desired_mode = "0";

            if(flymode)
            {
                desired_mode = "A";
            }

           	//set current location as desired position in offboard mode
           	DesiredPoseStamped.pose.position.x = current_x;
           	DesiredPoseStamped.pose.position.y = current_y;
        	DesiredPoseStamped.pose.position.z = 0.2;

            DesiredPoseStamped.pose.orientation.x = 0;
            DesiredPoseStamped.pose.orientation.y = 0;
            DesiredPoseStamped.pose.orientation.z = 0.7071;
            DesiredPoseStamped.pose.orientation.w = 0.7071;

            //set these as the starting x,y position
            start_x = current_x;
            start_y = current_y;
        }
        last_request = ros::Time::now();
    } 
}