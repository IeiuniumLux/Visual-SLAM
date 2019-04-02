#include <ModeHeader.h>

void mode_F()
{
	followWaypoint = true;
	desWaypoint = 0;

	//send to first waypoint
	desired_mode = "W" + to_string(desWaypoint+1);
}