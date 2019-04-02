#include <ModeHeader.h>

void mode_DISARM()
{
	ros::ServiceClient arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool arm_cmd;

  if(current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && disarmNow)
            {
              arm_cmd.request.value = false;
              //call mavros arm service and check if it was successful
              if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                  //display vehicle armed and change mode to takeoff
                    ROS_INFO("Vehicle DISARMED");
                    desired_mode = "0";
                    disarmNow = false;
                }
                last_request = ros::Time::now();
            } 
}