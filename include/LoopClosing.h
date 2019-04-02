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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

/* *
 * Loop closure.
 * This singleton object executes the LoopClosing :: Run method on a unique thread.
 * Each time a keyframe is added to the map (LoopClosing :: CheckNewKeyFrames), it tries to detect a loop (LoopClosing :: DetectLoop).
 * If detected, compute your pose (LoopClosing :: ComputeSim3), correct the map (LoopClosing :: CorrectLoop) using the final step LoopClosing :: SearchAndFuse.
 *
 * Finishing the thread, to close the application:
 * 1- An external thread invokes LoopClosing :: RequestFinish, which marks mbFinishedRequested
 * 2 - In the main LoopClosing :: Run loop, LoopClosing :: CheckFinish reports that completion has been requested and exits the loop
 * 3- Before finishing the thread, LoopClosing :: Run invokes LoopClosing :: SetFinish (), which marks LoopClosing :: mbFinished.
 * 4- LoopClosing :: isFinished () reports that mark to other threads that ask.
 *
 * Reset:
 * Upon a bad initialization, Tracking :: Reset invokes LoopClosing :: RequestReset, which blocks the Tracking thread until LoopClosing reinitializes.
 *
 */

class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    /* *
     * Unique constructor invoked by the System builder to create the singleton.
     */
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    /* *
     * Registers the system tracker.
     * @param pTracker pointer to singleton tracker system.
     *
     * Invoked only by System during its construction.
     *
     * The tracker pointer is not used.
     */
    void SetTracker(Tracking* pTracker);

    /* *
     * Registers the local mapper in LoopClosing :: mpLocalMapper.
     */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    /* *
     * Function that is executed in the thread dedicated to closing loops, and houses the main loop of this process.
     */
    void Run();

    /* *
     * Add new keyframes to the process queue to find loops.
     *
     * @param pKF new keyframe to process
     */
    void InsertKeyFrame(KeyFrame *pKF);

    /* *
     * Requests to reboot the process.
     *
     * Invoked by Tracking when map initialization fails.
     */
    void RequestReset();

    /* *
     * Bundle adjustment on the entire map after closing the loop.
     */
    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    /* *
     * @returns LoopClosing :: mbRunningGBA
     *
     * Invoked only by LoopClosing :: CorrectLoop and System :: Shutdown.
     */
    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    /* *
     * @returns LoopClosing :: mbFinishedGBA
     *
     * Only invoked by LoopClosing :: CorrectLoop
     */
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    /* *
     * Another thread requests completion of LoopClosing.
     */
    void RequestFinish();

    /* *
     * Tell other threads that LoopClosing is finished.
     */
    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    /* *
     * Indicates if there are keyframes in the queue waiting to be processed.
     *
     * @returns true if there are keyframes in the queue.
     */
    bool CheckNewKeyFrames();

    /* *
     * Process the keyframes of the queue, looking for loops.
     *
     * Produces a LoopClosing :: mvpEnoughConsistentCandidates list.
     */
    bool DetectLoop();

    /* *
     * Process the list of candidates trying to correct their pose.
     * If the login is not perfect, the candidates are discarded.
     * Successful inserts are then used to correct the loop.
     */
    bool ComputeSim3();

    /* *
     * Projects the observed points in the vicinity of the loop keyframe,
     * Over the current keyframe and neighbors using the corrected poses.
     * Merges duplicates.
     *
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    /* *
     * Corrects the loop with the locks created by LoopClosing :: ComputeSim3.
     */
    void CorrectLoop();

    /* *
     * Reinitialize the object if there is a pending request to do so.
     * Invoked at each iteration of the LoopClosing :: Run main loop.
     */
    void ResetIfRequested();

    /* *
     * Reboot Request Mark.
     * Only the tracker requests the reinitialization of the singleton LoopClosing, when the initialization of the map failed.
     */
    bool mbResetRequested;

    /* *
     * Mutex to access LoopClosing :: mbResetRequested
     */
    std::mutex mMutexReset;

    /* *
     * @returns LoopClosing :: mbFinishRequested
     * In the main LoopClosing :: Run loop,
     * LoopClosing :: CheckFinish informs if completion has been requested.
     */
    bool CheckFinish();

    /* *
     * True LoopClosing :: mbFinished, for querying other threads.
     */
    void SetFinish();

    /* *
     * Mark that LoopClosing has been requested to terminate.
     */
    bool mbFinishRequested;

    /* *
     * Completed process mark.
     */
    bool mbFinished;

    /* *
     * Mutex to access LoopClosing :: mbFinished.
     */
    std::mutex mMutexFinish;

    /* * Singleton map pointer. */
    Map* mpMap;
    Tracking* mpTracker;

    /* * Pointer to the database of singleton keyframes. */
    KeyFrameDatabase* mpKeyFrameDB;

    /* * Pointer to BoW vocabulary singleton. */
    ORBVocabulary* mpORBVocabulary;

    /* *
     * Pointer to singleton local mapper.
     * The loopback process interacted with the local mapper only to request pause and resume,
     * So as not to interfere with the map.
     */
    LocalMapping *mpLocalMapper;

    /* *
     * List of new keyframes to analyze for loops.
     *
     * LocalMapping creates keyframes and adds them to this list using LoopClosing :: InsertKeyFrame.
     * LoopClosing :: DetectLoop processes them on a first-come, first-served basis.
     */
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    /* *
     * Mutex to access the queue of new keyframes LoopClosing :: mlpLoopKeyFrameQueue
     */
    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    // Variable Loop Detector
    /* *
     * "Current" keyframe, extracted from the keyframes queue to be processed.
     * Ephemeral.
     */
    KeyFrame* mpCurrentKF;

    /* *
     * Keyframe macros with LoopClosing :: mpCurrentKF.
     * LoopClosing :: ComputeSim3 finds the keyframe and registers it in this variable.
     * Ephemeral.
     */
    KeyFrame* mpMatchedKF;

    /* *
     * Vector keyframes and their membership group.
     * Ephemeral.
     */
    std::vector<ConsistentGroup> mvConsistentGroups;

    /* *
     * List of keyframes candidates to close a loop.
     * Ephemeral vector.
     */
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;

    /* *
     * Set of keyframes connected by the graph of covisibilidad.
     * Used in a loop closure, to correct poses and graphs.
     * Vector ephemeral used only in LoopClosing :: CorrectLoop.
     */
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;

    /* *
     * Set of points macheados for the loop, in LoopClosing :: ComputeSim3.
     * LoopClosing :: CorrectLoop uses it to close the loop.
     * It is a vector ephemeral.
     */
    std::vector<MapPoint*> mvpCurrentMatchedPoints;

    /* *
     * Set points observed by the current keyframe and its neighbors, to consider and merge in the loop closure.
     *
     * It is defined as complete in LoopClosing :: ComputeSim3, and is used in LoopClosing :: SearchAndFuse.
     * It is a vector ephemeral.
     */
    std::vector<MapPoint*> mvpLoopMapPoints;

    /* *
     * Camera position relative to the world, proposed for the current keyframe when closing the loop, by LoopClosing :: Computesim3,
     * That uses it to search for more macheos with ORBmatcher.SearchByProjection.
     */
    cv::Mat mScw;

    /* *
     * Position of the camera with respect to the world in g2o format, from where LoopClosing :: mScw is obtained.
     */
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    /* *
     * Mark indicating that a Global Bundle Adjustement is running.
     * It is marked at the end of Loopclosing :: CorrectLoop and set to false at the end of Loopclosing :: RunGlobalBundleAdjustment.
     * Is reported by Loopclosing :: isRunningGBA.
     */
    bool mbRunningGBA;

    /* *
     * Mark exactly opposite mbRunningGBA.
     * Do not add information.
     */
    bool mbFinishedGBA;

    /* *
     * Mark to request abort the Global Bundle Adjustment, because it is necessary to start another with better data.
     */
    bool mbStopGBA;

    /* *
     * Mutex to access mbRunningGBA and mbFinishedGBA.
     * LoopClosing :: RunGlobalBundleAdjustment stops queries isFinishedGBA and isRunningGBA while propagating the result
     * Of the GBA to the new keyframes that could have arisen during its computation.
     *
     * These "is" queries are assumed to be performed by the local mapper.
     */
    std::mutex mMutexGBA;

    /* *
     * Thread for the Global Bundle Adjustment.
     */
    std::thread* mpThreadGBA;

    /* *
     * Always false in monocular.
     */
    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
