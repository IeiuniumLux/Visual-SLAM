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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/split_free.hpp>



#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

/* *
 * Each instance represents a 3D map in the map reference system.
* MapPoint is much more than the coordinates of the 3D point. It contains, for example, the list of keyframes that observe it.
 * Implements an ABC (high, low, query) of "observations" (keyframes), and another of associated descriptors.
 * Even choose the best descriptor, the closest to the rest of its descriptors.
 * Among other properties, a MapPoint has a normal that tries to indicate from which side is observable.
* It also has a range of distances in which it is observable. From very near or far the descriptors do not machean.
 * This range reduces the mache scope to increase performance.
 * The reference keyframe is the one entered in the constructor, presumably the first keyframe that observes the point.
 * If the reference keyframe is deleted, the next one is adopted from the list.
 * The point bears the number of times that "should have been observed" according to the pose of the camera,
 * Of the number of times it was actually found (its descriptor was macheted).
 *
 * There are three sets of ephemeral properties for specific use of tracking, loop closing, and local mapping.
 * These properties hold values ​​associated with the point and of short duration, valid throughout a procedure.
 * They are not multithreaded, the procedures begin by assuming some value that they consume later in the same procedure, and it makes no sense when finalizing.
 */
class MapPoint
{
public:

    MapPoint();			/* Default constructor for serialization */

	/* *
	 * Constructor that takes arguments values.
	 *
	 * @param Pos Position on the map.
	 * @param pRefKF reference keyframe.
	 * @param pMap Map to which the point belongs. There is a single map in ORB-SLAM.
	 * @param rgb_ Color of the point. Optional, black if not provided.
	 *
	 * This constructor is invoked only from LocalMapping :: CreateNewMapPoints for SLAM point aggregation.
	 *
	 * Only builder used.
	 *
	 */
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    /* *
     * Assigns to the point the 3D coordinates argument.
     *
     * @param Pos Vector with the position to be copied to the point.
     */
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    /* *
     * Assigns to the point the 3D coordinates argument.
     *
     * @param Pos Vector with the position to be copied to the point.
     */
    void SetWorldPos(const cv::Mat &Pos);

    /* *
     * Devote a Mat with the coordinates of the point.
     *
     * @returns Vector point position.
     */
    cv::Mat GetWorldPos();


    /* *
     * Average normal vector of observations.
     * Points are usually on a surface, and can only be seen on one side.
     * Each point approaches its normal vector as an average of the observations.
     *
     * @returns Normal point vector.
     */
    cv::Mat GetNormal();

    /* *
     * Returns the reference keyframe.
     *
     * @returns reference keyframe.
     */
    KeyFrame* GetReferenceKeyFrame();

    /* *
     * Sets the reference keyframe.
     *
     * @param pRefKF reference keyframe.
     */
    std::map<KeyFrame*,size_t> GetObservations();

    /* * Reports the number of observations recorded by the point. */
    int Observations();

    /* * Each time the point is observed from a keyframe, that observation is recorded.
     * @param pKF Keyframe that observes the point.
     * @param idx index of the keyframe that observes the point.
     */
    void AddObservation(KeyFrame* pKF,size_t idx);

    /* *
     * Removes from the point the record indicating that it was observed by that keyframe.
     *
     * Deletes the keyframe from the map mObservations and decrements the number of observations of the point.
     * If the removed keyframe is the reference, choose another.
     * If the point is observed by only 2 keyframes, it eliminates it with SetBadFlag.
     *
     * @param pKF Keyframe to delete.
     *
     * If pKF does not observe the point, it does nothing.
     */
    void EraseObservation(KeyFrame* pKF);

    /* *
     * Index of this point in the vector of features of the keyframe. -1 if the keyframe does not observe this point.
     *
     * @param pKF Keyframe to test, to see whether or not to observe the point.
     *
     * Returns the index idx, so this point is pKF-> mvpMapPoints [idx]
     */
    int GetIndexInKeyFrame(KeyFrame* pKF);

    /* *
     * Search if this point is observed by the keyframe argument.
     *
     * @param pKF Keyframe where to look for the point.
     * @returns true if you found the dot.
     */
    bool IsInKeyFrame(KeyFrame* pKF);

    /* *
     * Remove the point, marking it as bad.
     * Removes it from the map.
     * It does not destroy the point itself, to avoid conflicts between threads, but it should.
     * This flag is ephemeral, although by some error some points marked as bad and removed from the map remain in other containers.
     *
     * With mutex marks mbBad and removes map mObservations.
     * Out of the mutex it runs the keyframes of mObservations to eliminate the point of its macheos.
     * Finally removes the map registration point.
     */
    void SetBadFlag();

    /* *
     * Report flag mBad.
     *
     * All iterators query this flag before considering the point.
     * On a vector, first ask if the pointer is null, and then if isBad.
     */
    bool isBad();

    /* * Takes the properties of the argument point. */
    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    /* * Increments the count of times the point was observed. */
    void IncreaseVisible(int n=1);
    /* * Increments the count of times the point was found. */
    void IncreaseFound(int n=1);

    /* * Percentage of times the point was detected, over the total number of times it was in the fustrum. */
    float GetFoundRatio();

    /* * Number of times the point was found. */
    inline int GetFound(){
        return mnFound;
    }

    /* *
     * Choose the best descriptor among all the keyframes that observe the point.
     * Retrieves all descriptors, computes distances between all combinations,
     * And choose the descriptor with the least average distance to the rest.
     * Save the value in mDescriptor.
     */
    void ComputeDistinctiveDescriptors();

    /* *
     * Returns the best 3D point descriptor.
     * A 3D point has several observations, and therefore several descriptors.
     * MapPoint :: ComputeDistinctiveDescriptors computes the one that best represents it.
     *
     * @returns 3D Point Descriptor.
     */
    cv::Mat GetDescriptor();

    /* *
     * Recalculates the normal vector and the depth of visibility from the observations.
     */

    void UpdateNormalAndDepth();

    /* *
     * Calculates the minimum distance at which the point can be observed.
     * It is calculated as 80% of the shortest distance at which it was actually observed.
     *
     * @returns Minimum viewing distance.
     */
    float GetMinDistanceInvariance();

    /* *
     * Calculates the maximum distance at which the point can be observed.
     * It is calculated as 127% of the greatest distance at which it was actually observed.
     *
     * @returns Maximum viewing distance.
     */
    float GetMaxDistanceInvariance();

    /* *
     * Predict the scale (the level of the pyramid) in which the point will be, starting from the observation distance.
     *
     * @param currentDist Current distance.
     * @param logScaleFactor Scale factor.
     * @returns Level of the pyramid.
     */
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);
    // int PredictScale(const float &currentDist, const float &logScaleFactor);
  	void SetMap(Map* map);
	  void SetObservations(std::vector<KeyFrame*>);

public:

    /* * Point identifier. Unique for each instance. */
    long unsigned int mnId;

    /* * Identifier for the next new point. */
    static long unsigned int nNextId;

    /* *
     * Id of the first keyframe that observes this point, whose value is assigned in the constructor and never modified.
     * Usually the reference keyframe.
     * In the rare case where the reference keyframe is deleted, the reference changes, but this id is preserved.
     *
     * It is used only in LocalMapping :: MapPointCulling to avoid discarding points from the keyframe that generated it or the following two.
     */
    long int mnFirstKFid;

    /* * Id of the first frame that observes this point. */
    long int mnFirstFrame;


    /* * Number of times it was observed. */
    int nObs;


    /* *
     * Ephemeral variables used by Tracking.
     *
     * Several are written only in Frame :: IsInFrustum and used in OrbMatcher :: SearchByProjection
     */
    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;

    /* *
     * Cosine of triangulation.
     */
    float mTrackViewCos;

    /* *
     * Variable used in Tracking :: UpdateLocalPoints.
     *
     * Initializes at 0 in the constructor, and is written and read only in Tracking :: UpdateLocalPoints.
     * After the load can be initialized to zero.
     *
     */
    long unsigned int mnTrackReferenceForFrame;

    /* *
     * Record the id of the last frame that observed the point.
     *
     * Used in Tracking only to distinguish whether it is being watched in the current frame or not.
     *
     * It is not necessary to serialize, the constructor initializes it to zero.
     */
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;


    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing

    /* *
     * Id of the last keyframe that corrected the position of the point.
     * Initialized at 0 in construction.
     * Written by CorrectLoop, which then invokes OptimizeEssentialGraph and reads it.
     */
    long unsigned int mnLoopPointForKF;

    /* *
     * Id of the last keyframe that corrected the position of the point.
     * Initialized at 0 in construction.
     * Written by CorrectLoop, which then invokes OptimizeEssentialGraph and reads it.
     */
    long unsigned int mnCorrectedByKF;

    /*
     * Written by CorrectLoop, which then invokes OptimizeEssentialGraph and reads it.
     */
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;

    /* *
     * RunBundleAdjustment assigns and then consumes it.
     */
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;
protected:

     // Position in absolute coordinates
	/* * Position in absolute coordinates of the global map.
     * Opencv Mat.
     */
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
	/* *
	 * Keyframes that observe this point, and their indexes.
	 *
	 * The map indexes pointers to keyframes that observe this point.
	 * The secondary data associated with the index is the index of the Vector KeyFrame :: mvpMapPoints, whose element is a pointer at this point.
	 */
     std::map<KeyFrame*,size_t> mObservations;
     std::map<long unsigned int, size_t> 	   mObservations_nId;

     // Mean viewing direction
	/* * Normal vector, computed as the average of the addresses of all point views. */
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
	/* * Best point descriptor. */
     cv::Mat mDescriptor;

     // Reference KeyFrame
	/* *
	 * Reference keyframe.
	 * Used only to close loops.
	 */
     KeyFrame* mpRefKF;

     // Tracking counters
	/* * Number of times the point was captured by the camera, regardless of whether it was detected.
	 * It is the number of times that "should have been seen", depending on the pose of the camera.
	 */
     int mnVisible;
	/* * Number of times it was visible, and could be detected. */
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
	/* * Deletion flag. The deleted points are not removed from the vector, to avoid rearming the vector. */
     bool mbBad;
     MapPoint* mpReplaced;

	/* Range of distances (minimum and maximum) in which it can be observed and recognized by its descriptor. */
     // Scale invariance distances
	/* * Minimum distance at which it can be observed and recognized by its descriptor. */
     float mfMinDistance;

	/* * Maximum distance at which it can be observed and recognized by its descriptor. */
	 float mfMaxDistance;

	/* * Map to which it belongs.
	 * There is a single map, the global map.
	 */
	 Map* mpMap;

	 std::pair<long unsigned int, bool> mref_KfId_pair;

	 //id_map mref_KfId_map;
	 std::mutex mMutexPos;
     std::mutex mMutexFeatures;


	 friend class boost::serialization::access;
 template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
	{
		boost::serialization::split_member(ar, *this, version);
	}




	template<class Archive>
	void save(Archive & ar, const unsigned int version) const;


	template<class Archive>
	void load(Archive & ar, const unsigned int version);

};

} //namespace ORB_SLAM



#if 1

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)

namespace boost {
namespace serialization {

/*** CV KeyFrame ***/
template<class Archive>
void serialize(Archive & ar, cv::KeyPoint & kf, const unsigned int version)
{
  ar & kf.angle;
  ar & kf.class_id;
  ar & kf.octave;
  ar & kf.response;
  ar & kf.response;
  ar & kf.pt.x;
  ar & kf.pt.y;

}



/*** Mat ***/
/** Serialization support for cv::Mat */
template<class Archive>
void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
{
  size_t elem_size = m.elemSize();
  size_t elem_type = m.type();

  ar & m.cols;
  ar & m.rows;
  ar & elem_size;
  ar & elem_type;

  const size_t data_size = m.cols * m.rows * elem_size;

  ar & boost::serialization::make_array(m.ptr(), data_size);
}

/** Serialization support for cv::Mat */
template<class Archive>
void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
{
  int cols, rows;
  size_t elem_size, elem_type;

  ar & cols;
  ar & rows;
  ar & elem_size;
  ar & elem_type;

  m.create(rows, cols, elem_type);
  size_t data_size = m.cols * m.rows * elem_size;

  ar & boost::serialization::make_array(m.ptr(), data_size);
}



}
}
#endif


#endif // MAPPOINT_H
