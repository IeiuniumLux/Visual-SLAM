/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <string> 
#include <sstream>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <opencv2/core/core.hpp>
#include "Converter.h"
#include <boost/bind.hpp>

#include "../../../include/System.h"

#include "ModeHeader.h"

using namespace std;


//*************************GLOBAL VARIABLES*****************************
string desired_mode = "0";
float current_x = 0.0, current_y = 0.0, current_z = 0.0;
double current_yaw = 0.0;
geometry_msgs::Quaternion rotatedQ;
geometry_msgs::PoseStamped CurrentPoseStamped;

float scale;
bool useRangefinder;

bool isTracked = false;
bool armNow = false, disarmNow = false;
bool followWaypoint = false;
bool flymode = false;
bool publishFromGCS = false;

mavros_msgs::State current_state;

geometry_msgs::PoseStamped DesiredPoseStamped;
float start_x = 0.0, start_y = 0.0;

int desWaypoint = 0;
int nWaypoints = 0;
vector<float> xWaypoint, yWaypoint, zWaypoint, yawWaypoint, tWaypoint;
float threshWaypointDist, threshWaypointYaw;
double waypoint_timer = 0.0, waypoint_start_time = 0.0;
bool waypoint_reached = false;

float takeoff_height;

ros::Time prevTrackedTime;
ros::Time last_request;

ros::NodeHandle * nh;

class ImageGrabber
{
	public:
	ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);
	void AltMessageReceived(const std_msgs::Float64& AltMsg);
	void PublishPose(cv::Mat Tcw);

	ORB_SLAM2::System* mpSLAM;
	ros::Publisher* pPosPub;
};

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    if(!Tcw.empty())
    {
		double current_roll, current_pitch;

    	isTracked = true;
		cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
       
		current_x = scale*twc.at<float>(0);
		current_y = scale*twc.at<float>(2);
		if(useRangefinder==false)
		{
			current_z = -scale*twc.at<float>(1);
		}

		tf::Quaternion quat(q[2], q[0], q[1], q[3]);
		tf::Matrix3x3 m(quat);
		//m.getRPY(current_pitch, current_yaw, current_roll);
		m.getRPY(current_roll, current_pitch, current_yaw);

		//ROS_INFO("q1: %0.2f, q2: %0.2f, q3: %0.2f, q4: %0.2f, cy: %0.2f", q[0], q[1], q[2], q[3], current_yaw);

		rotatedQ = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -current_yaw+M_PI/2);		

       	prevTrackedTime = ros::Time::now();
    }
	return;
}

void ModeMessageReceived(const std_msgs::String& ModeMsg) 
{
	desired_mode = ModeMsg.data;
	ROS_INFO("New Desired Mode: %s", desired_mode.c_str());

	//should be false by default. switched to true in mode_SEARCH.cpp
	publishFromGCS = false;

	//should be false by default. switched to true in mode_F.cpp
	followWaypoint = false;

	if(desired_mode.compare("A")==0)
	{
		armNow = true;
	}
	else if(desired_mode.compare("DISARM")==0)
	{
		disarmNow = true;
	}

	return;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	return;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat Tcw= mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	PublishPose(Tcw);
return;
}

void ImageGrabber::AltMessageReceived(const std_msgs::Float64& AltMsg)
{	
	//Update z position using rangefinder altitude
	if(useRangefinder==true)
	{
		current_z = AltMsg.data;
	}
	//publish current pose
	CurrentPoseStamped.header.frame_id = "VSLAM";
	CurrentPoseStamped.header.stamp = ros::Time::now();

	CurrentPoseStamped.pose.position.x = current_x;
	CurrentPoseStamped.pose.position.y = current_y;
	CurrentPoseStamped.pose.position.z = current_z;

	CurrentPoseStamped.pose.orientation = rotatedQ;
	(pPosPub)->publish(CurrentPoseStamped);
	//PosPub.publish(CurrentPoseStamped);
	return;
}

void check_waypoint_distance()
{
	float distX = (current_x - xWaypoint[desWaypoint-1])*(current_x - xWaypoint[desWaypoint-1]);
    float distY = (current_y - yWaypoint[desWaypoint-1])*(current_y - yWaypoint[desWaypoint-1]);
    float distZ = (current_z - zWaypoint[desWaypoint-1])*(current_z - zWaypoint[desWaypoint-1]);

    double current_FLU_yaw = -current_yaw*180/M_PI;
    if(yawWaypoint[desWaypoint-1] > 90 && yawWaypoint[desWaypoint-1] <= 180)
    {
    	if(current_FLU_yaw < -90 && current_FLU_yaw >= -180)
    	{
    		current_FLU_yaw = current_FLU_yaw + 360;
    	}
    }
    else if(yawWaypoint[desWaypoint-1] <-90 && yawWaypoint[desWaypoint-1] >= -180)
    {
    	if(current_FLU_yaw > 90 && current_FLU_yaw <= 180)
    	{
    		current_FLU_yaw = current_FLU_yaw - 360;
    	}
    }    
    float diffYaw = fabs(current_FLU_yaw - yawWaypoint[desWaypoint-1]);

    if((distX + distY + distZ)<(threshWaypointDist*threshWaypointDist) && diffYaw<threshWaypointYaw)
    {
    	if(waypoint_reached == false)
    	{
    		waypoint_reached = true;
    		waypoint_start_time = ros::Time::now().toSec();
    	}
    	waypoint_timer = ros::Time::now().toSec() - waypoint_start_time;

		if(waypoint_timer > tWaypoint[desWaypoint-1])
		{
   			if(desWaypoint < nWaypoints)
   			{
   				desired_mode = "W" + to_string(desWaypoint+1);    			
   			}
   			else
   			{	
   				followWaypoint = false;
    			desired_mode = "L";
   			}
			waypoint_reached = false;
   		}
	}
}

void check_desired_mode()
{
	if(desired_mode.compare("O")==0)
		mode_O();
   	//mode to takeoff
   	else if(desired_mode.compare("T")==0)
    	mode_T();
    //mode to land
    else if(desired_mode.compare("L")==0)
	    mode_L();
  	//mode to arm vehicle
   	else if(desired_mode.compare("A")==0)
		mode_A();
    //mode to disarm vehicle
    else if(desired_mode.compare("DISARM")==0)
    	mode_DISARM();
    //mode to change desired position to home
    else if (desired_mode.compare("H")==0)
       	mode_H();
   	//mode to record waypoint
   	else if (desired_mode.compare(0,1,"R")==0)
   		mode_R();
   	//mode to change desired position to specific waypoint
  	else if (desired_mode.compare(0,1,"W")==0)
  		mode_W();
   	//mode to change desired waypoint to next waypoint
   	else if (desired_mode.compare("N")==0)
		mode_N();
   	//mode to automatically follow recorded waypoints
   	else if (desired_mode.compare("F")==0)
   		mode_F();
   	//mode to search for bottles
   	else if (desired_mode.compare("SEARCH")==0)
   		mode_SEARCH();
   	//mode to clear waypoints
   	else if(desired_mode.compare(0,2,"CW")==0)
   		mode_CW();
   	//mode to load saved set of waypoints
   	else if (desired_mode.compare(0,3,"LW-") == 0)
   		mode_LW();
   	//mode to save set of waypoints
	else if (desired_mode.compare(0,3,"SW-") == 0)
		mode_SW();
	else if (desired_mode.compare("FLY")==0)
   		mode_FLY();
	else
	{
		ROS_ERROR("Desired mode %s rejected. Mode not recognized.", desired_mode.c_str());
		desired_mode = "0";
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono");
	ros::start();

	ros::NodeHandle nodeHandler;
	nh = &nodeHandler;

	bool withViewer;
	bool bReuseMap;
	bool isSaveMap;

	string camera_yaml;
	nodeHandler.param("/Mono/ORB_SLAM2/camera_yaml", camera_yaml, string("/Visual-SLAM/Examples/RGB-D/TUM1realsense.yaml"));
	camera_yaml = ros::package::getPath("ORB_SLAM2") + "/../../RGB-D/" + camera_yaml;

	string camera_topic;
	nodeHandler.param("/Mono/ORB_SLAM2/camera_topic", camera_topic, string("/camera/rgb/image_color"));

	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/scale", scale, float(1.0));
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/save_map", isSaveMap, false);
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/use_rangefinder", useRangefinder, true);
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/with_viewer", withViewer, true);
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/reuse_map", bReuseMap, false);
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/waypoint_dist_threshold", threshWaypointDist, float(0.3));
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/waypoint_yaw_threshold", threshWaypointYaw, float(10));
	nodeHandler.param("/Mono/ORB_SLAM2/additional_params/takeoff_height", takeoff_height, float(0.8));

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ROS_INFO("Loading SLAM ... ");

	string map_file = ros::package::getPath("ORB_SLAM2") + "/../../../Slam_latest_Map.bin";
	//ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, withViewer, bReuseMap);
	ORB_SLAM2::System SLAM(argv[1],camera_yaml,ORB_SLAM2::System::MONOCULAR, withViewer, bReuseMap, map_file);

	//imagegrabber class object
	ImageGrabber igb(&SLAM);

	//subscribed topics
	ros::Subscriber state_sub = nodeHandler.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber sub = nodeHandler.subscribe(camera_topic, 1, &ImageGrabber::GrabImage,&igb);
	ros::Subscriber sub_mode = nodeHandler.subscribe("/navigation_mode", 1000, &ModeMessageReceived);
	ros::Subscriber sub_alt = nodeHandler.subscribe("/rangefinder_altitude", 1000, &ImageGrabber::AltMessageReceived,&igb);

	//published topics
	ros::Publisher PosPub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
	ros::Publisher desired_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
	igb.pPosPub = &(PosPub);

	//timers
	last_request = ros::Time::now();
	prevTrackedTime = ros::Time::now();

	//count variable for desired pose header
	int count = 0;

	//Default frameid and desired orientation
   	DesiredPoseStamped.header.frame_id = 1;
  	DesiredPoseStamped.pose.orientation.x = 0;
   	DesiredPoseStamped.pose.orientation.y = 0;
   	DesiredPoseStamped.pose.orientation.z = 0.7071;
   	DesiredPoseStamped.pose.orientation.w = 0.7071;

    while(ros::ok())
    {
    	//topic timestamp and count
       	DesiredPoseStamped.header.stamp = ros::Time::now();
       	DesiredPoseStamped.header.seq=count;

       	//check if mode change is desired
       	if(desired_mode.compare("0")!=0)
       		check_desired_mode();

    	//Check if within threshold to desired waypoint
    	if(followWaypoint)
    		check_waypoint_distance();

    	//increment count for header.seq
    	count++;


       //Publish desired position when vision is working. isTracked flag stops publishing if tracking is lost.
		if(isTracked==true && publishFromGCS==false)
		{
			//publish desired position and reset flag
		    desired_pub.publish(DesiredPoseStamped); 
			isTracked = false;
		}

		//check if tracking is lost for more than 0.5 second
    	if((ros::Time::now()-prevTrackedTime).toSec() > 0.5 && desired_mode.compare("L")!=0)
    	{
    		ROS_ERROR("No vision pose available. Swtiching to Land!");
    		desired_mode = "L";
    	}

      	ros::spinOnce();
     
   }

    //Stop all threads
    SLAM.Shutdown();

    //Save map
    if(isSaveMap)
    {
    	SLAM.SaveMap(map_file);    	
    }

    //Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;

}