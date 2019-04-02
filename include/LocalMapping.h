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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

/* *
 * LocalMapping has a single instance running the Run method on its own thread.
 * It maintains the local map, adding keyframes when Tracking creates them,
 * And looking for new triangulating points from neighboring keyframes.
 *
 * Has two sets of methods: one for local map computation, one for thread interaction.
 * The latter handle finite automata of interaction, usually initiated from outside,
 * Such as via RequestReset, RequestStop or RequestFinish.
 *
 * Computer methods are protected and invoked from Run.
 *
 * There is no formal entity that represents the "local map".
 * The local map is a global map subset, rooted in the reference keyframe,
 * Involving the neighbors in the graph.
 *
 *
 * Public methods for interaction with the thread
 *
* The Run method has an infinite loop, which runs on its own thread. Communications with him are asynchronous:
 *
* - LocalMapping :: RequestStop prompts you to stop, pause. LocalMapping :: isStopped can be checked to confirm that it has stopped.
 * - Run enters a loop waiting for the signal to resume.
 * - LocalMapping :: Release prompts you to resume after a pause.
* - LocalMapping :: RequestReset reinitializes the object by cleaning variables. Invoked from Tracking :: Reset. The reset is asynchronous, there is no sign that it has already occurred.
* - LocalMapping :: RequestFinish requests to terminate. You can check with LocalMapping :: isFinished. In practice it does nothing.
 *
 *
 */

class LocalMapping
{
public:

	/* *
	 * Constructor of the single instance of LocalMapping.
	 * @param mMap Global map.
	 * @param bMonocular Signal indicating that the system is monocular.
	 */
    LocalMapping(Map* pMap, const float bMonocular);

    /* * Registers the LoopCloser, the only instance in the system .. */
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /* * Registers the tracker, the only instance on the system. */
    void SetTracker(Tracking* pTracker);

    // Main function
 /* *
     * Main loop of the local mapping thread.
     * Receive and process control orders, such as reset, finish, etc.
     * When keyframes are in the list of new keyframes, it processes them by triggering the main mapping tasks in this order:
     *
     * -ProcessNewKeyFrame
     * -MapPointCulling
     * -CreateNewMapPoints
     * -SearchInNeighbors
     * -Optimizer :: LocalBundleAdjustment
     * -KeyFrameCulling
     *
     * If there are several new keyframes in the list, process from one to one by loop.
     * Some of the main tasks listed do not run until the list of new keyframes is emptied.
     */
    void Run();

    /* *
     * Method used by the tracker to request the inclusion of a new keyframe.
     * This method adds the keyframe to the LocalMapping.mlNewKeyFrames queue, which is processed in a separate thread,
     * In LocalMapping.Run by LocalMapping.ProcessNewKeyFrame.
     *
     * Summoned only from Tracking.
     */
    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    /* *
     * Pause request.
     * Other threads request to temporarily pause LocalMapping to interact with the map, generally for BA.
     * It is also invoked when it is passed to tracking mode only, without mapping.
     * After requesting the stop, they expect to be met by consulting LocalMapping :: isStopped.
     * To resume mapping, you must invoke LocalMapping :: Release.
     */
    void RequestStop();

    /* * Reboot request. Summoned only by Tracking :: Reset. */
    void RequestReset();

    /* * Invoked in Run's main loop, to process stop requests. If there is a stop request, it returns true. */
    bool Stop();

    /* * Invoked from other threads, cleans the buffer of new keyframes and resumes mapping. */
    void Release();

    /* * Reports whether LocalMapping is stopped. */
    bool isStopped();

    /* * Indicates whether a stop has been requested. */
    bool stopRequested();

    /* * Informs if LocalMapping accepts new keyframes. */
    bool AcceptKeyFrames();

    /* * Sets the acceptance of new keyframes. */
    void SetAcceptKeyFrames(bool flag);

    /* * Sets the non-stop signal, which ignores stop requests. Summoned from Tracking. */
    bool SetNotStop(bool flag);

    /* * Request interruption of BA. Summoned from Tracking. */
    void InterruptBA();

    /* * Request to terminate to close the system. */
    void RequestFinish();

    /* * Reports if LocalMapping is finished. */
    bool isFinished();

    /* * Reports the number of keyframes in the queue to be added to the map. */
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    /* *
     * Check the list of new keyframes mlNewKeyFrames.
     * @returns true if there are keyframes in the list.
     */
    bool CheckNewKeyFrames();

    /* *
     * Updates LocalMapping.mpCurrentKeyFrame, and removes the new keyframe from the LocalMapping.mlNewKeyFrames list.
     * For each 3D point displayed computes BoW and recompute descriptors, normal and depth.
     *
     * Summoned only from LocalMapping.Run.
     */
    void ProcessNewKeyFrame();

    /* *
     * The local mapping cycle periodically invokes this method, which searches candidate macheos for their triangulation and added to the map.
     * The search is done by triangulating points of the current keyframe and all its neighbors in the graph.
     * Invokes SearchForTriangulation to obtain the matched pairs.
     * Then evaluate the parallax to discard the point.
     * Finally triangulate are SVD and add it to the map.
     * Performs this operation for the current keyframe, compared to each of its neighbors in the graph.
     */
    void CreateNewMapPoints();

    /* *
     * Removes newly added points for various reasons.
     * Eliminates them if the point is bad, or very little observed.
     * Invoked from the main loop of LocalMapping :: Run.
     */
    void MapPointCulling();

    /* *
     * Scroll through neighboring keyframes for points to merge.
     *
     * This is the only place invoking ORBmatcher :: Fuse
     * Walk through the first and second order neighbors.
     * SearchInNeighbors runs only LocalMapping :: Run just finishes processing all new keyframes.
     *
     */
    void SearchInNeighbors();

    /* *
     * Eliminates redundant keyframes, which do not add information.
     * Search the local map only and remove keyframes with more than 90% of their points observed by other keyframes.
     * Invoked from the main loop of LocalMapping :: Run.
     */
    void KeyFrameCulling();

    /* *
     * Compute the fundamental matrix F of the keyframe 1 with respect to the keyframe 2.
     * @param pKF1 First keyframe
     * @param pKF2 Second keyframe
     * @returns The fundamental matrix F12.
     *
     * Summoned only from CreateNewMapPoints.
     */
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    /* *
     * Generates an antisymmetric matrix from a vector.
     * An array is antisymmetric when its transpose is its negative.
     *
     * @param v Vector 1x3.
     * @returns Antisymmetric Matrix of 3x3.
     */
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    /* * Monocular mode signal. */
    bool mbMonocular;

    /* * Process the restart request. */
    void ResetIfRequested();
    /* * Reboot requested. */
    bool mbResetRequested;
    std::mutex mMutexReset;

    /* * Reports if finished. */
    bool CheckFinish();
    /* * Mark the signals as finished. */
    void SetFinish();
    /* * Termination requested. */
    bool mbFinishRequested;
    /* * Finished. */
    bool mbFinished;
    std::mutex mMutexFinish;

    /* * World map. */
    Map* mpMap;

    /* * Loop closer. */
    LoopClosing* mpLoopCloser;
    /* * Tracker. */
    Tracking* mpTracker;

    /* *
     * Buffer request new keyframes.
     *
     * FIFO queue, new orders are added at the end with push_back,
     * And the keyframe obtained from front is processed.
     */
    std::list<KeyFrame*> mlNewKeyFrames;

    /* * Current keyframe, the last new keyframe that has been processed. */
    KeyFrame* mpCurrentKeyFrame;

    /* * List of new points added to the map, which will be reviewed by LocalMapping :: MapPointCulling */
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    /* * BA aborted. */
    bool mbAbortBA;
    /* LocalMapping stopped. */
    bool mbStopped;
   /* * Pause requested. */
    bool mbStopRequested;
    /* * Do not stop signal. */
    bool mbNotStop;
    std::mutex mMutexStop;

    /* *
     * Signal to accept new keyframes.
     * Written with LocalMapping :: SetAcceptKeyFrames, and read with LocalMapping :: AcceptKeyFrames.
     * Read only by Tracking :: NeedNewKeyFrame.
     * Written in the main loop of LocalMapping :: Run,
     * So that it is set to false at the beginning (indicating that the mapping does not accept new keyframes),
     * And true at the end, just before bed for 3 sec.
     */
    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
