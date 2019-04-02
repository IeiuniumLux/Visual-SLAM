#include <ModeHeader.h>

void mode_SW()
{

			ofstream saveFile;
			//string filename = "waypoint_files/" + desired_mode.substr(3,desired_mode.length()-3) + ".txt";
			string filename = ros::package::getPath("ORB_SLAM2") + "/../../../waypoint_files/" + desired_mode.substr(3,desired_mode.length()-3) + ".txt";
			saveFile.open(filename);
			
			for (int i = 0; i < nWaypoints; i++)
			{
				saveFile << xWaypoint[i] << "," << yWaypoint[i] << "," << zWaypoint[i] << "," << yawWaypoint[i] << "," << tWaypoint[i] << endl;
			}
			
			ROS_INFO("Saved file: %s with %d waypoints", filename.c_str(), nWaypoints);
			saveFile.close();
			desired_mode = "0";

}
