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

#ifndef KEYFRAME_H
#define KEYFRAME_H
#include <iostream>
using namespace std;
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/serialization/split_member.hpp>
#include <mutex>


namespace ORB_SLAM2
{
struct id_map
{
	bool is_valid;
	long unsigned int id;
};
class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
struct id_map;


/* *
 * Keyframe, keyframe.
 * ORB-SLAM identifies some frames as a key and adds them to the map.
 * A KeyFrame has more data than a Frame. When constructed copy the data of the Frame.
 * Frame is ephemeral, the KeyFrame remains on the map.
 * LocalMapping triangulates new map points exclusively from keyframes, not frames.
 * Tracking :: CreateNewKeyFrame has the exclusive creation of keyframes.
 *
 * Matrices
 *
 * - E: essential matrix, 3x3
 * - F: fundamental matrix, 3x3
 * - H: homography, 3x3
 * - K: intrinsic, chamber or calibration matrix, 3x3
 * - R: rotation matrix, 3x3, derived from a pose
 * - T: transformation, homogeneous matrix of 4x4, roto-structuring, to express poses
 *
 * Usual subindices:
 *
 * - Tcw: "Camera pose regarding World"
 * - F21: "fundamental matrix of table 2 with respect to 1"
 *
 * Vectors:
 *
 * - t: translation, derived from a pose
 *
 * Coordinates origin:
 *
 * The origin of coordinates by default is the world, whose origin is given by the pose of the first camera,
 * And the scale by the distance between the initialization chambers that triangulated the first points.
 *
 *
 *
 */

class KeyFrame
{
public:
	/* *
	 * Constructor that copies the data of the frame that is converted into keyframe.
	 *
	 * @param F Frame reference, which is erected in KeyFrame
	 * @param pMap Local map where this KeyFrame is located, necessary only to remove the KeyFrame from the map.
	 * @param pKFDB Database of keyframes where all keyframes are registered, only necessary to remove this KeyFrame from the database. It is registered in mpKeyFrameDB.
	 *
	 * This constructor copies all Frame F values.
	 * Invoked only from:
	 * - Tracking :: CreateInitialMapMonocular to create the first two keyframes, from the two frames used in the triangulation of the first points of the map.
	 * - Tracking :: CreateNewKeyFrame to create all other keyframes, always from the current mCurrentFrame box.
	 *
	 */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);


    KeyFrame();	/* Default constructor for serialization */
    
	// Pose functions
    /* *
     * Set Tcw, the pose of the keyframe.
     *
     * @param Tcw Array of camera pose regarding the world.
     *
     * This method also calculates other forms of expression of the pose:
     * - Twc
     * - Rcw
     * - Rwc
     * - tcw
     * - Ow
     *
     */
    void SetPose(const cv::Mat &Tcw);
	/* Tcw, the pose of the keyframe. @returns Tcw, pose. */
    cv::Mat GetPose();
	/*Twc, the inverse matrix of the pose. @returns Twc, the reverse of the pose. */
    cv::Mat GetPoseInverse();
	/*Reads the vector center of camera, equal to the translation vector for monocular. @returns the position vector of the camera, equal to the translation. It is different only in stereo. */
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();

	/* *
     * Read the 3D rotation matrix, obtained from Tcw.
     * @returns R, 3D rotation matrix 3x3, part of the pose.
     *
     * The rotation matrix rotates a vector
     */
    cv::Mat GetRotation();

	/* * Reads the translation vector, obtained from Tcw. */
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    /* *
     * Connects the keyframe to another in the graph of transparency.
     * The graph of clavisibility is a map of keyframe pairs and weight KeyFrame :: mConnectedKeyFrameWeights.
     *
     * @param pKF keyframe to connect.
     * @param weight Weight or weight of the connection.
     *
     * AddConnection adds another keyframe to the visibility graph of this keyframe.
     *
     * Invoked only from KeyFrame :: UpdateConnections.
     */
    void AddConnection(KeyFrame* pKF, const int &weight);

    /* *
     * Removes from the graph of covisibilidad the connection with the given keyframe.
     *
     * @param pKF keyframe to disconnect.
     */
    void EraseConnection(KeyFrame* pKF);

	/* *
     * Relieves coexistence and creates connections in the graph of covisibility.
     *
     * Create KeyFrame :: mvpOrderedConnectedKeyFrames, KeyFrame :: mvOrderedWeights, and KeyFrame :: mConnectedKeyFrameWeights.
     *
     * Scroll through all the mapPoints seen by this keyframe, and relieves all other keyframes that observe them,
     * Creating the KeyFrame :: mConnectedKeyFrameWeights covisibility map.
     *
     * It adds itself to the map of each relieved keyframe, invoking its respective KeyFrame :: AddConnection methods.
     * This action should ensure double reference.
     *
     * It is invoked by:
     *
     * ORB_SLAM2 :: Tracking :: CreateInitialMapMonocular (): void (2 matches), when creating lso first keyframes of the initial map.
     * ORB_SLAM2 :: LocalMapping :: ProcessNewKeyFrame (): void, when the KeyFrame is created (asynchronously).
     * ORB_SLAM2 :: LocalMapping :: SearchInNeighbors (): void
     * ORB_SLAM2 :: LoopClosing :: CorrectLoop (): void (3 matches), closing a loop must redo the graph of coexistence
     *
     * Perhaps loop closure could interfere with a simultaneous call from part of LocalMapping. That is why he uses mutex.
     *
     * Regarding the possible overlaps are SetBadFlag:
     *
     * LoopClosing marks the keyframe with SetNotErase. That brand persists in SearchInNeighbors, so there is no possible interference.
     * ProcessNewKeyFrame creates the keyframe, it is impossible at that time to be marked as bad. There is also no interference.
     *
     */
    void UpdateConnections();
	
	 /* *
     * Updates the best covisibles.
     *
     * The graph of covisibility is a map.
     * This method produces two aligned vectors, KeyFrame :: mvpOrderedConnectedKeyFrames with keyframes,
     * And KeyFrame :: mvOrderedWeights with their respective weights.
     *
     * Only uses KeyFrame :: mConnectedKeyFrameWeights data.
     *
     *
     * Invoked each time the graph is updated (each adding or deleting a connection).
     */
    void UpdateBestCovisibles();

	/* * Returns a set of keyframes connected to it by the essential graph. */
    std::set<KeyFrame *> GetConnectedKeyFrames();
	
	/* * Returns the vector of keyframe keyframes KeyFrame :: mvpOrderedConnectedKeyFrames. */
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();

	/* * Returns the first and best N elements of the KeyFrame :: mvpOrderedConnectedKeyFrames keyframes vector. @param N Maximum number of keyframes to return. */
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

	/* * Returns the best vector elements of cov- erable keyframes, with better weight than the reference. @param w Reference weight. */
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
	/* * Reports the weight of the connection of a certain cov- erable keyframe. @param pKF Keyframe whose weight you want to know. @returns Weight of the keyframe. */
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions

	/* *
     * Adds a child keyframe to the KeyFrame :: mspChildrens keyframe graph.
     * @param pKF child keyframe to add.
     *
     * Summoned only since
	 * - ORB_SLAM2 :: KeyFrame :: ChangeParent
	 * - ORB_SLAM2 :: KeyFrame :: UpdateConnections
     */
    void AddChild(KeyFrame* pKF);
	/* * Delete a child keyframe from the KeyFrame :: mspChildrens graph. @param pKF child keyframe to erase. */
    void EraseChild(KeyFrame* pKF);
    
	/* *
     * Change the parent of the keyframe.
     *
     * @param pKF New parent keyframe.
     *
     * Invoked only from ORB_SLAM2 :: KeyFrame :: SetBadFlag (), when mending the graph.
     */	
	void ChangeParent(KeyFrame* pKF);

	/* * Returns the children of this keyframe. @returns Keyframes children. */
    std::set<KeyFrame*> GetChilds();

	/* * Reports the parent keyframe. @returns Keyframe priest. */
    KeyFrame* GetParent();

	/* * Indicates if a keyframe is the child of the keyframe. @param pKF Keyframe presumed son. @returns true if child, false if not. */
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
	 /* * Adds an axis to the covisibility graph, from a loop detection. @param pKF Keyframe with which to stretch the axis. */
    void AddLoopEdge(KeyFrame* pKF);

	/* * Informs the keyframes connected with looping axes. @returns Keyframes connected by loop axes. */
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions

	 /* *
     * Add a 3D point to the list of observed points.
     *
     * @param pMP 3D point to add to the keyframe.
     * @param idx Index of the point in the 3D point vector, also index of the associated singular point.
     *
     *
     */
    void AddMapPoint(MapPoint* pMP, const size_t &idx);

	 /* *
     * Replace the index point with NULL in the mvpMapPoints vector.
     *
     * @param idx Index of the point to delete.
     *
     * It is a final function, it is limited to replacing the element indicated by NULL.
     *
     * Invoked only by MapPoint :: Replace and MapPoint :: SetBadFlag.
     *
     */
    void EraseMapPointMatch(const size_t &idx);

	 /* *
     * Remove the point from the list of points observed by this keyfame.
     * Replace the point with NULL in the mvpMapPoints vector.
     *
     * @param pMP 3D point to remove from the observed 3D points vector.
     *
     * If you do not find it, do nothing.
     * To erase the point, the keyframe must be able to be found in your observations. If it is not found, the point is not erased.
     *
     * It is a final function, it does not seek consistency with other containers.
     *
     * Invoked only from ORB_SLAM2 :: Optimizer :: LocalBundleAdjustment to delete outliers.
     */	
    void EraseMapPointMatch(MapPoint* pMP);

	/* *
     * Replaces an element of observer points with a new 3D point.
     *
     * @param idx Index of the point to replace.
     * @param pMP New point that replaces the previous one.
     *
     */
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);

	/* *
     * Returns the 3D points observed by the keyframe.
     * Traverses the vector KeyFrame :: mvpMapPoints and generates a set, copying the non-NULL elements.
     * @returns 3D point set.
     */
    std::set<MapPoint*> GetMapPoints();

	/* * Returns the 3D point vector. @returns KeyFrame :: mvpMapPoints. */
    std::vector<MapPoint*> GetMapPointMatches();

	/* *
     * Indicates the number of 3D points that have at least a given number of observations.
     *
     * @param minObs Minimum number of observations
     * @returns Number of 3D points with at least that minimum amount of observations.
     */
    int TrackedMapPoints(const int &minObs);

	/* *
     * Returns the 3D point from the index.
     *
     * @param idx Index for vector KeyFrame :: mvpMapPoints.
     * @returns 3D Point.
     */
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
	
    /* *
     * Filter singular points that are in the center square x, yy side 2 r.
     *
     * @param x Coordinate of the center of the square.
     * @param and Coordinate of the center of the square.
     * @param r Half-length of the square.
     * @returns Vector indices of singular points in the square area.
     */
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
	
	
    cv::Mat UnprojectStereo(int i);

    // Image
	 /* *
     * Checks if the coordinate is within the area of ​​the image.
     * @param x Coordinate x.
     * @param and y coordinate.
     * @returns true if the coordinate is within the image area, false if not.
     */
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
	/* *
     * Prevents deletion of keyframe while using loop detection.
     *
     * Makes mbNotErase = true.
     *
     * LoopClosing prevents the deletion of the keyframes being considered by marking them with KeyFrame :: SetNotErase before processing them,
     * And releasing them at the end with KeyFrame :: SetErase.
     *
     * Invoked only from LoopClosing :: DetectLoop and LoopClosing :: ComputeSim3.
     * DetectLoop invokes it only on mpCurrentKF
     */
    void SetNotErase();

	 /* *
     * Free the keyframe previously marked with KeyFrame :: SetNotErase.
     *
     * Makes mbNotErase = false, only if the keyframe is not connected by looping axes to other keyframes.
     *
     * Seize the moment and check if the keyframe is marked for deletion, in which case it invokes SetBadFlag.
     *
     * SetErase is a step before SetBadFlag, invoked only from LoopClosing.
     * Repeatedly invoked only from LoopClosing :: DetectLoop and LoopClosing :: ComputeSim3 to free the keyframe.
     *
     * DetectLoop only invokes it on mpCurrentKF.
     *
     * These two methods mark keyframes not to erase and release them at the end with KeyFrame :: SetErase,
     * Except that the keyframe ends up forming part of a loop closure, in which case it will be marked forever.
     */
    void SetErase();

    // Set/check bad flag

	/* *
     * Try to delete the keyframe and mark it as bad. With this mark the algorithms will ignore it.
     *
     * It is allowed to delete the keyframe:
     * - is removed from the map
     * - is removed from KeyFrameDatabase
     * - the keyframes graph is patched by choosing new parents for the children.
     * - removed from observations of each MapPoint of mvpMapPoints
     * - is marked mbBad
     *
     *
     * The object itself is never deleted, although it should. It is not released because it has no way of ensuring that no one is accessing it.
     * One way would be to create a vector of orphan keyframes with a timestamp of their decease, and compare it with timestamps of threads that could use it.
     *
     * The mbBad flag is used when accessing a keyframe that is just erasing from another thread.
     *
     * If deletion of that keyframe (mbNotErase == true) is prevented, it is marked for immediate deletion as soon as it is released:
     * KeyFrame :: Erase releases the keyframe, verifies the mbToBeErase tag, and if true, invokes SetBadFlag again.
     * This elimination prevention system is used only during the loop detection process, on a temporary basis.
     * When a keyframe is locked in a loop, this system will prevent its deletion forever.
     *
     * It has no effect on the initial keyframe, with id 0.
     *
     *
     * SetBadFlag blocks mMutexConnections and mMutexFeatures, and marks mbBad before unlocking.
     * Strangely methods that use this lock do not query the mbBad mark before proceeding.
     * This could explain the bug that lingers in the graph some keyframes marked as bad.
     *
     *
     * Invoked only by LocalMapping :: KeyFrameCulling and KeyFrame :: SetErase.
     *
     * ALL:
     * It would be best to remove the keyframe from the map first, in order to reduce the likelihood that other threads will find it.
     * Similarly, it should be marked Bad from the beginning.
     *
     */
    void SetBadFlag();

	 
	/* *
     * Check the mbBad brand.
     * True if it is bad.
     * When the keyframe is bad, it is ignored without exceptions at all orb-slam2.
     * It would mean removing the instance, but that can break some ephemeral pointer.
     * Serialization is a good time to remove the bad guys.
     *
     * If this flag is true, the keyframe has been removed from Map, KeyFrameDatabase, all mapPoint and spanning tree observations.
     *
     */

    bool isBad();


    /* *
     * Compute the depth of the midpoint of the scene.
     * The scene is the observer set of 3D points by the keyframe.
     *
     * The depth of a 3D point relative to a camera pose is the distance from that point to the plane parallel to the camera image, which passes through its focus.
     *
     * Calculates the depth for each point in the world, sorts them by depth and returns the depth of point 1 / q.
     *
     * It would fail if there were no 3D point in the scene.
     *
     * @param q Fractionator, q> = 1, which chooses which depth to return. 1 returns the one of the most distant, infinite of the nearest, 2 the median.
     * @returns Depth of the chosen point.
     *
     * It is invoked from two places, always with the argument q = 2.
     *
     * From Tracking :: CreateInitialMapMonocular this depth is used to establish the unit of measure of the world.
     * From LocalMapping :: CreateNewMapPoints is used to discard the triangulation of a pair of keyframes.
     */
    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

	/* *
     * Compare two weights.
     * @param a Weight a.
     * @param b Weight b.
     * @returns a> b.
     */

    static bool weightComp( int a, int b){
        return a>b;
    }

    /* *
     * Compare Id. Smaller id, greater antiquity.
     * @param pKF1 keyframe 1.
     * @param pKF2 keyframe 2.
     * @returns true if pKF1 is older than pKF2.
     */
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

	void SetMap(Map* map);
	void SetKeyFrameDatabase(KeyFrameDatabase* pKeyFrameDB);
	void SetORBvocabulary(ORBVocabulary* pORBvocabulary);
	void SetMapPoints(std::vector<MapPoint*> spMapPoints);
	void SetSpanningTree(std::vector<KeyFrame*> vpKeyFrames);
	void SetGridParams(std::vector<KeyFrame*> vpKeyFrames);



    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

	/* * Counter for self-enumeration of ids. */
    static long unsigned int nNextId;

	/* * Id of the keyFrame. The id is a sequence number. */
    long unsigned int mnId;

	/* * Id of the associated frame. The id is a sequence number of the frames. */
    const long unsigned int mnFrameId;

	 /* * Time stamp. */
    const double mTimeStamp;

	/* * Dimensions of frame grid to accelerate macheo. They are given by mnGridCols and mnGridRows. */
    // Grid (to speed up feature matching)
    const int mnGridCols;

	 /* * Dimensions of frame grid to accelerate macheo. They are given by mnGridCols and mnGridRows. */
    const int mnGridRows;

	/* * The inverse of the width of the grid in pixels. Retrieved from the associated Frame. */
    const float mfGridElementWidthInv;

	/* * The inverse of the height of the grid in pixels. Retrieved from the associated Frame. */
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)

    /* * Unique points visualized by the keyframe. */
    const std::vector<cv::KeyPoint> mvKeys;

	/* * Unique points with "undistorted" coordinates. The elements correspond to those of mvKeys. */
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points

	/* * Descriptors. They match those of mvKeys. */
    const cv::Mat mDescriptors;

    //BoW
	/* * BoW vector obtained from the keyframe descriptors. ComputeBoW fills this vector. */
    DBoW2::BowVector mBowVec;
	/* * DBoW2 Features Vector. ComputeBoW fills this vector. */
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
	/* * Pose of camera relative to his father. */
    cv::Mat mTcp;

    // Scale
	/* * Number of levels of the pyramid. */
    const int mnScaleLevels;
	/* * Scale factor between two consecutive levels of the pyramid. */
    const float mfScaleFactor;
	/* * Logarithm of the scale factor. */
    const float mfLogScaleFactor;
	/* * Absolute scale factors for every level of the pyramid. */
    const std::vector<float> mvScaleFactors;
	/* * Square Sigma (square of KeyFrame :: mvScaleFactors) for each level of the pyramid. */
    const std::vector<float> mvLevelSigma2;
	/* * Inverse of square sigma for each level of the pyramid. */
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
	/* * Vertices of the undistorted image: mnMinX, mnMinY, mnMaxX, mnMaxY. */
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
	 /* *
     * K Camera calibration matrix.
     */
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
	 /* *
     * Matrix of 4x4 of rototraslación in homogeneous coordinates, that expresses the pose of the keyframe.
     * SetPose determines its value, and GetPose reads it.
     * GetRotation and GetTraslation get the rotation matrix and the translation vector in Euclidean coordinates.
     *
     * Tcw is the transformation of camera coordinates (c) to world coordinates (w).
     * Your inverse is Twc.
     */
    cv::Mat Tcw;
	/* *
     * Matrix of 4x4 roto-structuring in homogeneous coordinates, with the inverse transformation of Tcw.
     * GetPoseInverse reads its value.
     */
    cv::Mat Twc;
	 /* *
     * Center of the camera.
     * Obtained with GetCameraCenter.
     */
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
	std::map<long unsigned int, id_map> 	   mmMapPoints_nId;

 	// BoW
	 /* * Database of keyframes, where this keyframe is located.
     * Received as constructor argument.
     * Used exclusively in SetBadFlag, to remove the keyFrame itself from the database.
     * This database is queried with a BoW descriptor to get the list of keyframes containing it.
     */
    KeyFrameDatabase* mpKeyFrameDB;
	 /* * BoW Vocabulary for ORB.
     * Represents a classification dictionary that returns a BoW from an ORB descriptor.
     */
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

	
	/* *
     * Map of coexistence, which links the covisible keyframes with their weights.
     * Completely generated via KeyFrame :: UpdateConnections, which invokes KeyFrame :: AddConnection.
     * It is generated from the observations of the map points of the keyframe.
     * When generated the same map of the other keyframes is updated.
     */
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
	 /* * Keyframes arranged by weight, updated via KeyFrame :: UpdateBestCovisibles. */
		std::map<long unsigned int, int> 	   mConnectedKeyFrameWeights_nId;
	 /* * Weights of KeyFrame keyframes :: mvpOrderedConnectedKeyFrames. */
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
		std::map<long unsigned int, id_map> 	mvpOrderedConnectedKeyFrames_nId;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges

    /* *
     * Status of the initialization controller.
     * True when the keyframe is constructed, false after the first connection is added,
     * Except it is the keyframe of id 0, which has no one to connect to and this mark always remains true.
     */
    bool mbFirstConnection;
	/* *
     * KeyFrame parent in the graph.
     */	
    KeyFrame* mpParent;
		id_map mparent_KfId_map;
	 /* *
     * KeyFrames children in the graph.
     *
     * It is populated as other keyframes identify this as a parent.
     *
     * The identification occurs in the first call to UpdateConnections as soon as the keyframe is created,
     * That traverses the keyframes with covisibles points, and chooses like father to which it shares more points.
     *
     * This implies that it is possible to rebuild this set by running UpdateConnections on each KeyFrame.
     */
    std::set<KeyFrame*> mspChildrens;
		std::map<long unsigned int, id_map> 	   mmChildrens_nId;

	/* *
     * KeyFrames that participate in one end of a loop.
     *
     * LoopClosing :: CorrectLoop is the only method that adds these axes,
     * And consuming them indirectly by invoking Optimizer :: OptimizeEssentialGraph.
     *
     * It is not ephemeral.
     *
     */
    std::set<KeyFrame*> mspLoopEdges;
		std::map<long unsigned int, id_map> 	   mmLoopEdges_nId;

    // Bad flags
	
    /* *
     * Do not erase signal.
     * True when an axis is added to the graph, or with SetNotErase () when the keyframe is used in loop detection.
     * Is set to false with SetErase (), only if it has no loop axis.
     *
     */
    bool mbNotErase;
	/* *
     * Mark the keyframe to be deleted.
     * Keyframes are built with this bit in false.
     * Set true only with SetBadFlag, when mbNotErase is true.
     * This flag indicates that the keyframe is pending deletion, could not be deleted because it had a loop axis
     * Or was being used in a loop detection at that time.
     *
     * Rebuildable: true if you have looping axes.
     */
    bool mbToBeErased;
	 /* *
     * Flag of elimination.
     *
     * When KeyFrame :: SetBadFlag can not undo a keyframe because it is marked with KeyFrame :: mbSetNotErase,
     * Set this flag to true, to retry the removal as soon as the above mentioned mark is removed.
     *
     * All the activities that involve a keyframe consult this flag before proceeding, using KeyFrame :: IsBad.
     *
     */
    bool mbBad;    

    float mHalfBaseline; // Only for visualization
	/* * Map where the keyFrame is located.
     * Received as constructor argument.
     * Used exclusively in SetBadFlag, to remove the keyFrame itself from the map.
     */

    Map* mpMap;

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

 	/* *
     * Mutex access to the connection graph.
     *
     * My add:
     * To prevent the keyframe from being used and registering just when it is being marked as bad with SetBadFlag,
     * The following methods that use this mutex and record the keyframe somewhere,
     * Will only proceed after checking the KeyFrame :: mbBad flag.
     * The methods with this aggregate are:
     *
     * - KeyFrame :: AddChild, prevents the keyframe from returning to the graph of readability
     * - KeyFrame :: AddConnection, prevents the keyframe from returning to the connection graph
     * - KeyFrame :: UpdateConnections, avoids reloading mConnectedKeyFrameWeights, mvpOrderedConnectedKeyFrames, and mvOrderedWeights
     *
     * - KeyFrame :: UpdateBestCovisibles will never be invoked if you just run SetBadFlag
     *
     * The ones invoked by LoopClosing are not included, as they have the power to require that the KeyFrame not be deleted.
     *
     *
     */
	
    std::mutex mMutexPose;
	/* *
     * Mutex access to mvpMapPoints.
     */
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;



};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
