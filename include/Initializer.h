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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

/* *
 * Map initializer in monocular mode.
 * Try to triangulate the first points of the map.
 *
 */

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:
	/* *
     * Constructor that takes ReferenceFrame as a reference for successive initialization attempts.
     *
     * @param ReferenceFrame Initial reference chart, the first of the two views required for triangulation.
     * @param sigma
     * @param iterations Maximum number of iterations for each triangulation attempt.
     */
    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

	/* *
     * Compute in parallel a fundamental matrix and homography.
     * Select the model with the best result and try to get the movement and the SFM (structure from motion) structure.
     * Try to triangulate the first points of the map.
     *
     * @param CurrentFrame Current frame.
     * @param vMatches12 Macheos.
     * @param R21 Matrix rotation of the second camera compared to the first, result of initialization.
     * @param t21 Vector translation of the second camera from the first, result of the initialization.
     * @param vP3D 3D triangulated points.
     * @param vbTriangulated Indicator of inliers (true) or outliers (false).
     * @returns true if triangulation succeeded, false if not.
     */

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:
	/* *
     * Looks for the homography between the two matched frames, using RANSAC.
     * Along with FindFundamental, these are the two initialization methods that try to get the initial pose.
     * All three arguments get only the result.
     * @param vbMatchesInliers Vector boolean whose elements indicate whether the corresponding singular points are inliers.
     * @param score Homography score. More better.
     * @param H21 Matrix of homographic transformation obtained.
     *
     */

    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);

    /* *
     * Compute the fundamental matrix between the two matched frames, using RANSAC.
     * Along with FindHomography, these are the two initialization methods that try to get the initial pose.
     * All three arguments receive the result.
     * @param vbInliers
     * @param score
     * @param F21 Fundamental matrix.
     */
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

 	/* *
     * Compute a homography for a given correspondence of normalized points in two images.
     *
     * @param vP1 Unique points in a box.
     * @param vP2 Matching singular points in the other frame.
     * @returns Matrix homography.
     *
     * Invoked only by Initializer :: FindHomography.
	 */
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

	 /* *
     * Compute the fundamental matrix for a given correspondence of normalized points in two images, using SVD.
     * @param vP1 Unique points in a box.
     * @param vP2 Matching singular points in the other frame.
     * @returns Matrix homography.
     *
     * Invoked only by Initializer :: FindFundamental.
	 */
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

	 /* *
     * Evaluates the quality of the homography, doing an epipolar verification.
     *
     * @param H21 Matrix of homographic transformation.
     * @param H12 Inverse of H21.
     * @param vbMatchesInliers Result, mark of singular points that passed the test.
     * @param Sigma Sigma sets the threshold for testing.
     * @returns Result of the evaluation. More better.
     *
     * Invoked only from Initializer :: FindHomography.
     */
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

	/* *
     * Evaluates the quality of the fundamental matrix, doing an epipolar verification.
     *
     * @param F21 Fundamental matrix.
     * @param vbMatchesInliers Result, mark of singular points that passed the test.
     * @param Sigma Sigma sets the threshold for testing.
     * @returns Result of the evaluation. More better.
     *
     * Invoked only from Initializer :: FindFundamental.
     */
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

	/* *
     * Verify the 4 roto-offing hypotheses obtained from the fundamental matrix.
     * If there are enough inliers and parallax, use that pose to initialize.
     *
     * @param vbMatchesInliers Mark of valid maches, inliers.
     * @param F21 Fundamental matrix.
     * @param K Matrix camera, intrinsic or calibration.
     * @param R21 Result: rotation matrix of one camera relative to the other.
     * @param t21 Result: vector translation of one camera relative to the other.
     * @param vP3D 3D Points.
     * @param vbTriangulated
     * @param minParallax Minimum parallax threshold: 1.0.
     * @param minTriangulated Minimum number of triangulated points: 50.
     * @returns true if you chose the candidate for initialization.
     *
     * Invoked only from Initializer :: initialize.
     */
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

	/* *
     * Checks the 8 hypothesis of roto-mapping of Faugeras obtained from homography.
     * If there are enough inliers and parallax, use that pose to initialize.
     *
     * @param vbMatchesInliers Mark of valid maches, inliers.
     * @param F21 Fundamental matrix.
     * @param K Matrix camera, intrinsic or calibration.
     * @param R21 Result: rotation matrix of one camera relative to the other.
     * @param t21 Result: vector translation of one camera relative to the other.
     * @param vP3D 3D Points.
     * @param vbTriangulated
     * @param minParallax Minimum parallax threshold: 1.0.
     * @param minTriangulated Minimum number of triangulated points: 50.
     * @returns true if you chose the candidate for initialization.
     *
     * Invoked only from Initializer :: initialize.
     */
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

	/* *
     * Triangulate 3D points from their views from both cameras.
     *
     * @param kp1 Unique point observed from camera 1.
     * @param kp2 Singular point observed from camera 2.
     * @param P1 Projection matrix of camera 1: K [I | 0]. 4x3 matrix based on the camera matrix.
     * @param P2 Camera projection matrix 2: K [R | t]. Matrix of 4x3 based on the camera matrix and the pose relative to the camera 1.
     * @param x3D Result, Euclidean coordinates of 3D point triangulated by SVD, in the coordinate system of camera 1.
     *
     * Use cv :: SVD to triangulate.
     *
     * The projection matrix P of a camera is a 4x3 matrix (3 rows, 4 columns), which combines its intrinsic and extrinsic matrices.
     * P = K [R | t], where K is the camera matrix, R and t is the rotation of the camera relative to the reference.
     * Premultiplying a homogeneous 3D coordinate (vector of 4 dimensions), its projection is obtained in homogeneous coordinates in pixels.
     *
     * Invoked only from Initializer :: CheckRT.
     */
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

	/* *
     * Produces points with the normalized coordinates of the singular points.
     * Normalize consists of placing the origin at the midpoint, and that the average distances to the origin are 1.
     *
     * @param vKeys Singular points to normalize.
     * @param vNormalizedPoints Result, points with normalized coordinates.
     * @param T Result, homogeneous matrix of 3x3, normalization transformation.
     *
     * Invoked only from Initializer :: FindHomography and Initializer :: FindFundamental.
     *
     */
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

	/* *
     * Check the pose to confirm H or F hypotheses.
     * Repeatedly invokes Initializer :: Triangulate to get the 3D points projecting the singular points.
     * Mark them as "good" if they do not have coordinates in infinity, they are in front of the cameras,
     * The reprojection error does not exceed the argument threshold and have under parallax (cosParallax <0.99998).
     *
     *
     * @param R Matrix rotation proposed.
     * @param t Vector translation proposed.
     * @param vKeys1 Singular points in table 1.
     * @param vKeys2 Singular points in table 2.
     * @param vMatches12 Macheos between singular points.
     * @param vnInliers Inlier signal.
     * @param K Matrix camera.
     * @param vP3D Result, triangulated 3D points.
     * @param th2 Square threshold, always 4 * sigma2, maximum reprojection error allowed for the point to be considered good.
     * @param vbGood Result, "good" triangulated dot signal, corresponding to vKeys1, vKeys2, vMatches12, vnInliers and vP3D.
     * @param parallax Score, parallax average.
     * @returns Number of good points.
     *
     * Invoked only from Initializer :: ReconstructF and Initializer :: ReconstructH.
     */
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

	/* *
     * Decompose the essential matrix E into rotation and vector translation matrices.
     *
     * @param E Essential Matrix.
     * @param R1 Result, rotation matrix 1 (first hypothesis).
     * @param R2 Result, rotation matrix 2 (second hypothesis).
     * @param t Result, vector translation for one hypothesis, its negative for the other.
     *
     * Invoked only from Initializer :: ReconstructF.
     */
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

	//Singular points in Table 1
    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

	//Singular points in Table 2
    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;
    vector<bool> mvbMatched1;

	//Calibration, intrinsic or camera matrix
    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

	 /* * Maximum number of iterations when applying RanSaC. */
    // Ransac max iterations
    int mMaxIterations;

	/* * Sets for Ransac. */
    // Ransac sets
    vector<vector<size_t> > mvSets;   

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
