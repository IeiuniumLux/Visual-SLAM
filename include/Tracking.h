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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

/* * Single object that runs on its own Trhead, and starts with Run ().
 * This object packages the code that performs the tracking and runs in its unique and exclusive thread.
 * .Run () hangs the listener GrabImage (), which receives messages with images from the camera node via the ros.
 *
 * By its asynchronous nature, GrabImage implements a finite automaton
 * Which leads the thread through the different initialization states, and relocation when the tracking is lost.
 * Its status variable is mState.
 *
 * Each cycle (each time an image is received) creates a Frame instance, where it saves the results of the
 * Detection of points, extraction of descriptors, macheo, link with 3D points, etc.
 *
 * In the Tracking thread, the initial triangulation, the SfM, is performed and the KeyFrames are generated.
 * Relocation is also triggered when needed.
 *
 * Build the local map in mvpLocalKeyFrames, a vector of KeyFrames.
 *
 * Convention of prefixes of members:
 * The properties adopt the following prefixes:
 * - m: member
 * - mp: pointer member
 * - mvp: pointer vector member
 * - mb: boolean member
* - mn:? Is used on some int members.
 *
 * Most of the methods are for internal use, and many of them are invoked only once,
 * In the sense that your code does not need to be in a method, but it is so to improve readability.
 *
 * Only three other objects use tracking methods:
 * - FramePublisher
 * - LocalMapping
 * - LoopClosing
 *
 * FramPublisher queries the states of Tracking mState and mLastState, uses its states enum,
 * And Update () takes data from the mCurrentFrame, mInitialFrame and mvIniMatches.
 *
 * LocalMapping :: SetTracker and LoopClosing :: SetTracker save tracking in mpTracking, and do not use it.
 *
 * Most object methods are parts of a single algorithm, divided for better understanding.
 * These methods are invoked only from another method of the object, and not from outside.
 *
 * The following sequence of methods correspond to a single algorithm that could be in a single function:
 *
 * GrabImageMonocular: Receive the image and convert it to grays
 * Track: executes the finite automaton
 * MonocularInitialization: look for maches to triangulate with PnP
 * CreateInitialMapMonocular: create the initial map with the triangulated points
 * TrackWithMotionModel: normal tracking, look for macheos according to the model of movement
 * UpdateLastFrame: trivial
 * TrackLocalMap - adds new points to the local map
 * SearchLocalPoints: updates the "visibility" (the counting of views) of the points, and filters points that are already on the map
 * UpdateLocalMap: Updates the local map: points and keyframes
 * UpdateLocalMapPoints: update the local map with new points
 * UpdateLocalKeyFrames: Updates local keyframes with new points
 * NeedNewKeyFrame: Evaluate if a new keyframe is needed
 * CreateNewKeyFrame: creates and registers a new keyframe from the current frame
 * TrackReferenceKeyFrame: alternative tracking, based on BoW, for normal tracking failure
 * CheckReplacedInLastFrame: updates points in lastKeyFrame that might have been changed by BA, culling or other reasons
 * Relocalization: invoked if the system lost tracking
 *
 */

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const bool bReuse);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);


    /* * Registers the LocalMapping object. @param pLocalMapper Local mapping object. */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /* * Registers the LoopClosing object. @param pLoopClosing Loop-closing object. */
    void SetLoopClosing(LoopClosing* pLoopClosing);

    /* * Registers the display object. @param pViewer Viewer. */
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    /* * Tracking status
    The mState state variable of the Tracking object has one of these values.
    */
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    /* * State of the finite tracking automaton. MState adopts the values ​​of the enum eTrackingState. */
    eTrackingState mState;

    /* * State previous to current. Last status was processed completely. */
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;

    /* *
     * Current frame image, newly converted to grays.
     * GrabImageMonocular receives the image as Mat, and if it is in color it converts it to grays; Save it in mImGray.
     */
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;

    /* * Vector with previous matches, for initialization. */
    std::vector<cv::Point2f> mvbPrevMatched;

    /* * Vector with the initial 3D points. */
    std::vector<cv::Point3f> mvIniP3D;

        /* *
     * First frame for initialization.
     * The triangulation of points for the initial map is done between two frames:
     * - mCurrentFrame
     * - mInitialFrame
     *
     * Once the system is initialized, mInitialFrame is not reused.
     */
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization

    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();


    /* *
     * Flag of visual odometry.
     * True when there are few macheos with map points.
     * The system enters visual odometry mode to guess where on the map it is, while trying to relocate.
     * The difference with normal tracking is that it only tries Tracking :: TrackWithMotionModel, and it does not try Tracking :: TrackReferenceKeyFrame.
     * Also, while performing VO, try to relocate.
     *
     * TrackWithMotionModel switches to visual odometry when it does not map (onlyTracking) and displays less than 10 map points.
     */
    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
        /* *
     * Extractors of camera.
     * Right is used only in stereo; Monocular uses only Left.
     * Ini is a more demanding version, which during the initialization tries to recover twice as many features and matches as normal.
     */
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    /* * List of local keyframes, part of the local map. */
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    /* * Local map, 3D point list. */
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    /* *
     * Rotation of the motion model.
     * It is a 4x4 matrix as Tcw, arises from multiplying Tcw of the current frame with Twc from the previous one.
     * The result is the relative rotation of the last table with respect to the previous one.
     */
    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
    list<MapPoint*> mlpTemporalPoints;
	bool is_preloaded;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
