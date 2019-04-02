#include <ModeHeader.h>

void mode_T()
{
  if(ros::Time::now() - last_request > ros::Duration(1.0))
  {
    ROS_WARN("TAKING OFF!");
    DesiredPoseStamped.pose.position.x = start_x;
    DesiredPoseStamped.pose.position.y = start_y;
    DesiredPoseStamped.pose.position.z = takeoff_height;

    desired_mode = "0";
  }
}