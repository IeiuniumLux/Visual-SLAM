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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

/* *
 * Map of the world, of points and keyframes.
 * The world map consists of a set of 3D points, 3D reference points, and keyframes that observe them.
 * ORB-SLAM uses a single map that is created in main.cc, it is passed to the constructors of the 3 threads (tracking, loopClosing, localMapping),
 * The MapPublisher constructor and framePublisher.
 * The map has a mbMapUpdates flag, protected, which indicates whether the map is updated or, on the contrary, there is some BA in process.
 * LocalMapping and loopClosing are the two threads that carry out BA, and when finished they correct the flag with SetFlagAfterBA ().
 * The flag may be false for very short periods in other updates, such as adding or deleting a point.
 */

class Map
{
public:
	/* * Constructor, which initializes properties. The map starts "outdated" with mbMapUpdated == false. */

    Map();

	/* * Add a new keyframe to the map.
	 * Registers the new keyframe in the vector keyframes of the map,
	 * And updates the mnMaxKFid record of the id of the last keyframe.
	 */
    void AddKeyFrame(KeyFrame* pKF);

    /* * Adds a 3D point to the map. */
    void AddMapPoint(MapPoint* pMP);

    /* *
     * Deletes a certain point on the map.
     *
     * Removes the point from the mspMapPoints container.
     * You should then delete the dot object, but do not to avoid concurrency conflicts.
     * This function is final, delegates the responsibility of data coherence to the function that invokes it.
     *
     *
     * @param pMP Map point to delete.
     *
     * Run mspMapPoints.erase (pMP)
     *
     * Invoked only from MapPoint :: Replace and MapPoint :: SetBadFlag.
     */
    void EraseMapPoint(MapPoint* pMP);

    /* * Deletes a particular keyframe from the map. */
    void EraseKeyFrame(KeyFrame* pKF);

    /* * Adds a point to the set of reference points.
     * This function is invoked exclusively from Tracing :: UpdateReference (), to record local map points here.
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    /* * Returns a vector of keyframes with all keyframes on the map. */
    std::vector<KeyFrame*> GetAllKeyFrames();

    /* * Returns a vector of points, with all points on the map. */
    std::vector<MapPoint*> GetAllMapPoints();

    /* *
     * Returns a vector of points to be plotted.
     * Invoked only by MapDrawer :: DrawMapPoints, these points are graphed in red, representing the local map.
     */
    std::vector<MapPoint*> GetReferenceMapPoints();

    /* * Reports the number of points on the map. */
    long unsigned int MapPointsInMap();

   /* * Reports the number of keyframes on the map. */
    long unsigned  KeyFramesInMap();

    /* * Reports the id of the last keyframe added. */
    long unsigned int GetMaxKFid();

    /* *
     * Restart the map.
     * Eliminates points and keyframes of the map (ie, the system), and cleans the vectors of points and keyframes.
     */
    void clear();

	/* *
	 * Registration of the initial keyframes.
	 * The only keyframe in this vector is added by Tracking :: CreateInitialMapMonocular. The initial keyframe.
	 * Is iterated by LoopClosing :: RunGlobalBundleAdjustment.
	 */
    vector<KeyFrame*> mvpKeyFrameOrigins;

    /* * Mutex for map update. */

    mutable std::mutex mMutexMapUpdate;

    /* * Mutex for adding new points to the map. */
    // This avoid that two points are created simultaneously in separate threads (id conflict)
    mutable std::mutex mMutexPointCreation;

protected:
    /* * Map points. */
    std::set<MapPoint*> mspMapPoints;

    /* * Map KeyFrames. */
    std::set<KeyFrame*> mspKeyFrames;

    /* *
     * Points of reference on the map.
     * They are the points of the local map, to graph in red.
     * Tracking :: UpdateReference generates it, and MapDrawer :: DrawMapPoints consumes it.
     * For each frame that is processed, the local map is updated and this vector is copied to the map.
     * It is a vector ephemeral.
     */
    std::vector<MapPoint*> mvpReferenceMapPoints;

    /* * Id of the last keyframe added. */
    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    /* * Mutext of the map. */
    std::mutex mMutexMap;

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
#endif // MAP_H
