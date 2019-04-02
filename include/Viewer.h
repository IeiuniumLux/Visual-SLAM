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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;


/* *
 * Viewer represents the user interface.
 *
 * It consists of two parts, implemented in two classes: FrameDrawer and MapDrawer, both singleton.
 *
* FrameDrawer is responsible for generating the processed image with the map points and a status bar. Viewer shows it with imshow.
 *
 * MapDrawer is responsible for loading the 3D points of the world into Pangolin, the keyframes and camera poses.
 *
 * These two objects prepare what is shown, the action of taking it to the screen occurs in Viewer :: Run by:
 * - imshow
 * - pangolin :: FinishFrame
 *
 * Pause:
 * The viewer process can be paused from another thread:
 * - Viewer :: RequestStop to request the pause
 * - Viewer :: isStopped to check if it is paused
 * - Viewer :: Release to remove the pause and resume the process
 *
 * During the pause the display is not updated.
 *
 *
 * Termination:
 * The process is definitely finished to close the application with:
 * - Viewer :: RequestFinish to request termination
 * - Viewer :: isFinished to confirm that it is finished
 */
class Viewer
{
public:

    /* *
     * Singleton singleton Viewer builder.
     * It deals with the user interface: presentation on screen and processing keys and buttons.
     * It is the only consumer of FrameDrawer and MapDrawer singletons.
     */
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, bool bReuse);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.

    /* *
     * Main Viewer method, which runs on its own thread.
     * Run enters an endless loop, which comes out only when asked to end with RequestFinish.
     * In this loop, Pangolin keyboards and buttons are processed, and images and map are displayed.
     */
    void Run();
    /* *
     * Request to terminate to close the application.
     * Summoned by another thread.
     */
    void RequestFinish();

    /* *
     * Prompt to pause.
     * Asynchronously invoked by another thread, which should be consulted isStopped to confirm that Viewer was stopped.
     * Viewer pauses in the Run loop, and resumes after another thread invokes Release.
     *
     */
    void RequestStop();

    /* *
     * Informs if Viewer is finished, previous step to close the application.
     *
     * @returns Viewer :: mbFinished: true if terminating.
     */
    bool isFinished();

    /* *
     * Returns true if Viewer is paused.
     * It is consulted repeatedly after having requested the stop with RequesStop.
     *
     * @returns Viewer :: mbStopped, which is true if Viewer :: Run is already in the pause loop.
     */
    bool isStopped();

    void Release();

private:

    bool Stop();
	bool mbReuse;
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

