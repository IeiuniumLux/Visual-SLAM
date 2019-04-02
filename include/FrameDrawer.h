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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

/**
 * Single instance class that deals with displaying the camera image, with green marks on the recognized points.
 * Use opencv imshow. At the bottom of the image it describes the state of the system.
 * FrameDrawer is accessed from two parallel asynchronous threads:
 * - Tracking :: Track invokes the FrameDrawer :: Update method warning that there is a new Frame to display.
 * - Viewer :: Run invokes the FrameDrawer :: DrawFrame method when it is time to update the screen.
 */
class FrameDrawer
{
public:
	//Builder
    FrameDrawer(Map* pMap, bool bReuse);

	//Summoned only by Tracking :: Track warning that there is a new Frame to display
    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

	
	//Only invoked by Viewer :: Run to send the image of the current frame to the screen
    // Draw last processed frame.
    cv::Mat DrawFrame();

protected:
	//Add a bar below the image and write the status of the system
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

	// Gray picture of the current frame
    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
	//Singular points of the current picture
    vector<cv::KeyPoint> mvCurrentKeys;

	/** Boolean vectors that indicate if the singular point is being tracked.
     * MvbMap indicates whether the singular point corresponds to a point on the map.
     * MvbVO indicates whether the singular point corresponds to a point not added to the map, for visual odometry.
     */
    vector<bool> mvbMap, mvbVO;

	//tracking only (true) or tracking & mapping (false)
    bool mbOnlyTracking;

	/* *
     * Number of matches with 3D points.
     * MnTracked indicates the number of matches between singular points in the frame and points on the map.
     * MnTrackedVO indicates the number of matches between singular points in the box and 3D points of visual odometry, not added to the map.
     */
    int mnTracked, mnTrackedVO;

	//Unique initialization points.
    vector<cv::KeyPoint> mvIniKeys;

	//Initialization macros
    vector<int> mvIniMatches;

	// the state of the finite system automaton
    int mState;
	
	//World map
    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
