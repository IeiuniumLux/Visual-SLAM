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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM2
{

/* *
 *
 * ORBmatcher packages all descriptor mache methods.
 * The few properties are protected, they correspond to the configuration established in the construction.
 * Not to be confused with class ORBextractor, of similar name, that packs all the methods of extraction of descriptors.
 * All methods are invoked from other objects.
 * Only DescriptorDistance, which computes the distance between two descriptors with the Stanford algorithm,
 * Is also invoked internally by virtually all other ORBMatcher methods.
 *
 * While ORBextractor takes care of detecting unique points and extracting ORB descriptors,
 * ORBmatcher takes care of machear in several ways:
 *
 * - SearchByProjection part of a pose, projects the local map points that should be visible, and performs a circular mache.
 * - SearchByBoW machea first by BoW, and then by descriptors when there are several candidates with the same BoW.
 * - SearchForInitialization machea to find the first points of the map.
 * - SearchForTriangulation machea to find new 3D points.
 * - SearchBySim3 machea to evaluate candidates for loop closure.
 * - Fuse merges duplicate map points.
 */
class ORBmatcher
{    
public:

    /* *
     * Constructor that saves the arguments in properties.
     * @param nnratio Nearest Neighbor ratio. It indicates the relationship between the best and worst candidate to deliver as a result.
     * @param checkOri Check Orientation. True to check the orientation of each point before evaluating the machete (true by default). It also has a history of orientations.
     *
     */
    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    /* *
     * Matches circular between singular points detected in the current picture, and the projection of the points of the map that should be seen.
     * I guess it avoids reprocessing the points that were already seen in LastFrame and that have been assigned to the current frame in TrackWithMotionModel.
     * Add new points to the record of points viewed from the current mvpMapPoints box.
     *
     * @param F Current picture
     * @param vpMapPoints Local Map
     * @param th Threshold, threshold.
     *
     * Uniquely invoked from Tracking :: SearchLocalPoints.
     */
    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    /* *
     * Matches ad hoc circular between single point descriptors of the current frame
     * With the singular points of the previous picture associated to some point of the local map.
     * Creates the list of local map points viewed from the CurrentFrame.mvpMapPoints box.
     *
     * @param CurrentFrame Current frame, with its unique points detected.
     * @param LastFrame Previous frame, with its unique points and its associated local map points.
     * @param bMono Flag that indicates whether it is monocular (true) or not. In this modified version (os1) it is always monocular.
     *
     * 1. Apply the motion model to predict a pose.
     * 2. Projects map points detected in LastFrame.
     * 3. Machea descriptors between the singular points of CurrentFrame found in a circular region with center in the projection.
     * 4. Associates the 3D points macheados to the singular points in CurrentFrame.
     *
     * Invoked exclusively from TrackWithMotionModel, which from this macheo computes the pose of the current frame.
     */
    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    /* *
     * Look in the current box for the points seen in the candidate keyframe for relocation.
     *
     * @param CurrentFrame Current frame.
     * @param pKF KeyFrame candidate for relocation.
     * @param AlreadyFound Map of points found.
     * @param Th Threshold, radio pattern for circular mache. The radius computed in pixels varies for each singular point according to the level of the pyramid.
     * @param ORBdist Maximum distance accepted between descriptors. If the best macheo has greater distance, it is discarded.
     *
     * Uniquely invoked from Tracking :: Relocalization.
     */
    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    /* *
     * In the presence of a loop detection with macros and a proposed pose, this method increases the number of macheos
     * Comparing against other points that should be viewed from the current keyframe.
     *
     * @param pKF Current keyframe, candidate to close loop.
     * @param Scw Roto-scale and scale (sim3), proposed pose transformation for loop closure.
     * @param vpPoints Possibly observed points according to the calculated pose, to be considered in the loop closure.
     * @param vpMatched Points machete in loop detection.
     * @param th threshold.
     *
     * Uniquely invoked from LoopClosing :: ComputeSim3.
     */
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    /* *
     * Matches points the current frame with those of a keyframe.
     * Matches by BoW and then by descriptors.
     *
     * @param pKF Keyframe candidate to explain the current frame. It is the keyframe of reference for tracking, or one of several candidates in relocation.
     * @param F Current frame you are trying to locate.
     * @param vpMapPointMatches Result, 3D points macheados.
     * @returns Number of points scored.
     * Invoked only by Tracking :: Relocalization and Tracking :: TrackByReferencingKeyFrame.
     */
    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);


    /* *
     * Evaluate candidate keyframe for loop closure.
     * Machea by BoW and then by descriptors.
     * @param pKF1 Current Keyframe.
     * @param pKF2 Keyframe candidate for loop closure.
     * @param vpMatches12 Search results, map points mapped.
     * @returns Number of machetes.
     * Only called from LoopClosing :: ComputeSim3.
     */
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);


    /* *
     * Mache for initialization.
     * This method looks for macheos in a loose window, looking for a high number of macheos that will later be considered for the triangulation of the first points of the map.
     * The method is invoked repeatedly and successively, frame by frame, until it is initialized, or until the amount of machete is reduced too much.
     * @param F1 Initial map of the map initialization process.
     * @param F2 Current frame.
     * @param vbPrevMatched Coordinates of the singular points macheados in the previous table (that is, the last time this method was invoked).
     * @param vnMatches12
     * @param windowSize Size of the search area, windowSize is the length of the side of the square area. It's always 100.
     * @returns Number of machetes obtained.
     *
     * Summoned only by Tracking :: MonocularInitialization.
     */
    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    /* *
     * Machea points unique between the current keyframe and a neighbor,
     * For the later purpose of triangulating new points and adding them to the map.
     * For each single point of the first keyframe, the macheus runs through the epipolar line of the second keyframe.
     * Uses the fundamental matrix to apply epipolar geometry, a way to check the geometric compatibility of the mache.
     * In addition it reduces the macheo effort by comparing by BoW before calculating distances of descriptors.
     *
     * @param pKF1 Current Keyframe.
     * @param pKF2 Keyframe neighbor.
     * @param F12 Fundamental matrix of pKF1 for pKF2.
     * @param vMatchedPairs Result of the algorithm, the matched pairs. If he had something, he erases it.
     * @param bOnlyStereo Flag that indicates whether it is only processed for stereo.
     * @returns Number of machetes found.
     *
     * Invoked only from LocalMapping :: CreateNewMapPoints
     */
    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);


    /* *
     * Compares two candidate keyframes to loop closure.
     *
     * @param pKF1 Current Keyframe.
     * @param pKF2 Keyframe candidate to close the loop.
     * @param vpMatches12 3D points seen from both cameras.
     * @param s12 Scale of pFK1 relative to pKF2.
     * @param R12 Matrix rotation of pFK1 with respect to pKF2.
     * @param t12 Translation of pFK1 to pKF2.
     * @param th Radio in pixels where you look for singular points to machear.
     * @returns Number of points macheados and explained by Sim3.
     *
     *
     * Only called from LoopClosing :: ComputeSim3.
     */
    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    /* *
     * Project map points over a keyframe and look for duplicate points as part of the local mapping.
     * Merge points on the map that correspond to the same real point.
     * It is used twice: projecting the points of the current keyframe over neighboring keyframes, and vice versa.
     * @param pKF Keyframe where the map points will be projected.
     * @param vpMapPoints Map points observed in a neighboring keyframe.
     * @param th Circular search radius.
     * @returns Number of points merged.
     *
     * Summoned twice only from LocalMapping :: SearchInNeighbors.
     */
    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    /* *
     * Merge the map points to close a loop.
     *
     * @param pKF Current keyframe or neighbor of one end of the closed loop.
     * @param Scw Pose sim3 from the camera regarding the world. This argument requires further investigation.
     * @param vpPoints Map points observed by the keyframe at the other end of the loop, and its neighbors.
     * @param th Radio for circular mache.
     * @param vpReplacePoint Method result, points merged.
     * @returns Number of points merged.
     *
     * Only invoked from LoopClosing :: SearchAndFuse
     */
    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:


    /* *
     * Low threshold, strict, for distances between descriptors.
     * `BestDist <= TH_LOW` is the usual check to accept or reject the best distance after comparing several candidate descriptors.
     * This threshold is used only inside the object, for all macheos except for machear descriptors of charts with map descriptors.
     * TH_LOW is set to 50 (up to 50 of the 256 bits of the descriptor may be different from the reference descriptor).
     */
    static const int TH_LOW;

    /* *
     * High threshold, lax, for distances between descriptors.
     * `BestDist <= TH_HI` is the usual check to accept or reject the best distance after comparing several candidate descriptors.
     * This threshold is used only within the object in ComputeSim3 and SearchByProjection for tracking.
     * TH_HIGH is set to 100 (up to 100 of the 256 bits of the descriptor may be different from the reference descriptor).
     */
    static const int TH_HIGH;

    /* *
     * Histogram array size.
     */
    static const int HISTO_LENGTH;


protected:


    /* * Check by epipolar line.
     * Check if a singular points of a pose are in (or very close to) the epipolar line of another singular point of another pose.
     *
     * @param kp1 Singular point of the pose 1.
     * @param kp2 Singular point of the pose 2.
     * @param F12 Fundamental matrix between both poses.
     * @param pKF1 Keyframe 1 (pose 1).
     * @param pKF2 Keyframe 2 (pose 2).
     * @returns A signal indicating that the points have epipolar correspondence.
     *
     * Summoned only from SearchForTriangulation, with singular points macheados.
     *
     */
    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    /* *
     * Package the radio decision in circular search, according to the triangulation cosine.
     * The higher parallax, the lower the radius, the search will be more strict.
     * Only called from SearchByProjection between keyframe and map.
     */
    float RadiusByViewingCos(const float &viewCos);

    /* *
     * Compute 3 maximums.
     * It traverses the vectors of the histogram, comparing its lengths, and returns the indexes of the three vectors of greater amount of elements.
     *
     * @param Histogram, array of int vectors. Always created as `vector <int> rotHist [HISTO_LENGTH];`.
     * @param L Length, amount of history elements. It's always HISTO_LENGTH.
     * @param ind1 Index of the longest histogram. Value result, one of the 3 maxima. It is always passed initialized to -1.
     * @param ind2 Index of second-longest histogram. Value result, one of the 3 maxima. It is always passed initialized to -1.
     * @param ind3 Third-largest histogram index. Value result, one of the 3 maxima. It is always passed initialized to -1.
     *
     * This method is invoked only from other methods of the same class. All of them initialize `int1 = int2 = int3 = -1`.
     */

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    /* *
     * Nearest neighbor ratio.
     * When looking for the two best distances, it is sought that the best is greater than a percentage (mfNNratio) of the second.
     * `BestDist1 <mfNNratio * bestDist2`
     * It is 0.6 by default.
     */
    float mfNNratio;

    /* *
     * Internal signal of configuration defined in the construction of the object, which indicates whether the orientation should be checked
     * Before comparing descriptors.
     * The orientation tells which side the descriptor is visible, so that with this check it is avoided to compare the 3D points observed from the opposite side.
     * Is `true` by default.
     */
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
