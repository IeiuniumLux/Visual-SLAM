#include <ModeHeader.h>

void mode_FLY()
{

	//change mode to offboard
	desired_mode = "O";

	//change flymode flag to true
	flymode = true;
}