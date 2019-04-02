#include <ModeHeader.h>

void mode_L()
{
	ros::ServiceClient set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	mavros_msgs::SetMode land_set_mode;
	land_set_mode.request.custom_mode = "AUTO.LAND";

  if(current_state.mode!="AUTO.LAND" && (ros::Time::now()-last_request > ros::Duration(5.0)))
  {
    if(set_mode_client.call(land_set_mode) && land_set_mode.response.success)
    {
      //display offboard mode and change it to default 0
	  ROS_INFO("LAND enabled");
      desired_mode = "0";
    }
  }
}