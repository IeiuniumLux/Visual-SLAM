#include <ModeHeader.h>

void mode_A()
{

  ros::ServiceClient arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool arm_cmd;

  //arm vehicle only current state is disarmed, and wait 5 seconds 
  if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && armNow)
  {
    arm_cmd.request.value = true;
    //call mavros arm service and check if it was successful
    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
      //display vehicle armed and change mode to takeoff
      ROS_INFO("Vehicle ARMED");
      desired_mode = "T";
      armNow = false;
      flymode = false;
    }
    last_request = ros::Time::now();
  }                  
}