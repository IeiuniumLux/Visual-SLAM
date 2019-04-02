#include <ModeHeader.h>


void mode_LW()
{
    xWaypoint.erase (xWaypoint.begin(), xWaypoint.begin() + nWaypoints);
    yWaypoint.erase (yWaypoint.begin(), yWaypoint.begin() + nWaypoints);
    zWaypoint.erase (zWaypoint.begin(), zWaypoint.begin() + nWaypoints);
    yawWaypoint.erase (yawWaypoint.begin(), yawWaypoint.begin() + nWaypoints);            
    tWaypoint.erase (tWaypoint.begin(), tWaypoint.begin() + nWaypoints);
    nWaypoints = 0;

	string filename = ros::package::getPath("ORB_SLAM2") + "/../../../waypoint_files/" + desired_mode.substr(3,desired_mode.length()-3) + ".txt";
	ifstream loadFile (filename);
	string line;

	if (loadFile.is_open())
	{
		while ( getline (loadFile,line) )
		{
			float temp;
			vector<float> vect;
			stringstream ssLine(line);
			while (ssLine >> temp)
			{
				vect.push_back(temp);
				if (ssLine.peek() == ',')
				{
					ssLine.ignore();
				}
			}
			xWaypoint.push_back(vect[0]);	yWaypoint.push_back(vect[1]);	zWaypoint.push_back(vect[2]);	yawWaypoint.push_back(vect[3]);     tWaypoint.push_back(vect[4]);
			nWaypoints++;
		}
		ROS_INFO("Loaded file: %s with %d waypoints", filename.c_str(), nWaypoints);
		loadFile.close();
	}
	desired_mode = "0";	
}
