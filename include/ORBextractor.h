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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    /* *
     * Divide the node into 4: n1, n2, n3 and n4.
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;

    /* *
     * Vertex of the bounding rectangle:
     * U: upper, B: bottom,
     * L: left, R: right
     *
     */
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;

    /* *
     * Signal indicating that the node has exactly one assigned point.
     * Initializes to false, several methods check and set it to true when it has a single point.
     */
    bool bNoMore;
};


/* *
 * Package all methods of detection of singular points and extraction of descriptors.
 * ORBextractor processes images with the operator ():
 * - detecting singular points
 * - computing your directions
 * - extracting their descriptors
 *
 * The object manages the pyramids and the grid of cells in which the image is divided to homogenize the distribution of singular points.
 * Tracking creates the only two instances of this object, long life, mpORBextractorLeft and mpIniORBextractor,
 * The first as an initial part of the tracking process in the OK state, and the second to initialize.
 *
 * The description of ORBextractor :: ComputeKeyPointsOctTree contains a good description of the singular points.
 */
class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /* *
     * Constructor that loads the received configuration values ​​as arguments, computes pyramid and precalculates factors.
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /* *
     * ORBextractor (...) processes images with the operator ():
     * - detecting singular points
     * - computing your directions
     * - extracting their descriptors
     *
     * @param image Image to process.
     * @param mask Mask. Not implemented.
     * @param keypoints Unique points detected as a result of the operation.
     * @param descriptors Descriptors extracted as a result of the operation.
     */
    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    /* *
     * Returns the protected attribute nlevels, the number of levels in the pyramid, set by the constructor and read-only.
     */
    int inline GetLevels(){
        return nlevels;}

    /* *
     * Returns the protected attribute scaleFactor, the scale factor between levels of the pyramid, set by the constructor and read-only.
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /* *
     * Returns the mvScaleFactor protected attribute.
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /* *
     * Returns the mvInvScaleFactor protected attribute.
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /* *
     * Returns the protected attribute mvInvLevelSigma2.
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }
    /* *
     * Returns the protected attribute mvInvLevelSigma2.
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }


    /* *
     * Images of the pyramid.
     * Produced by ComputePyramid, it is a vector of'nlevels' images.
     */
    std::vector<cv::Mat> mvImagePyramid;

protected:


    /* *
     * Generate the images of the pyramid and save them in the mvImagePyramid vector.
     * @param image Image to process, obtained from the camera.
     * Called only from the constructor.
     */
    void ComputePyramid(cv::Mat image);


    /* *
     * Detects unique points, and scatters them with ORBextractor :: DistributeOctTree.
     *
     * @param allKeypoints Vector of unique points to detect, result of the process. A vector of unique points for each level of the pyramid.
     *
     * The KeyPoints obtained store the following data:
     * - pt, coordinates of the singular point on the image.
     * - angle, orientation obtained with IC_Angle.
     * - octave, level of the pyramid.
     * - size, size according to the scale factor of the level of the pyramid.
     */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);  

    /* *
     * Distributes single points with an octTree.
     * Receives a number of singulae points much higher than desired, this method eliminates most so that the surviving points are scattered in the image homogeneously.
     * OcTree is a tree whose nodes have 8 children exactly, which abstractly represent 8 vertices of a cube.
     * Https://en.wikipedia.org/wiki/Octree
     *
     * @param vToDistributeKeys Unique points to distribute.
     * @param minX border, threshold, fringe image that is not parsed.
     * @param maxX Border, threshold, stripe of the image that is not parsed.
     * @param minY Border, threshold, fringe of image that is not parsed.
     * @param maxY Border, threshold, stripe of the image that is not parsed.
     * @param nFeatures Desired number of unique points.
     * @param level Level of the pyramid to process.
     * @returns Singular points distributed.
     */  
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /* *
     * Old method to detect singular points, replaced by ComputeKeyPointsOctTree.
     * Method not used.
     */

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /* *
     * Coordinates BRIEF.
     * Each row contains a pair of x coordinates, and whose intensities are compared to obtain a bit of the binary descriptor.
     * There are 256 rows for the 256 bits of BRIEF. All coordinates enter a circular patch of 31 pixels in diameter.
     * The coordinates are relative to the singular point to be considered, and to its orientation.
     * This means that these pattern coordinates are rotated to get the real coordinates to extract a descriptor.
     * They are defined first in
     * Static int bit_pattern_31_ [256 * 4] '
     */
    std::vector<cv::Point> pattern;

    /* *
     * Number of desired single points in an image.
     * Set in the ORB-SLAM2 configuration file.
     */
    int nfeatures;


    /* *
     * Scale factor between levels of the pyramid.
     * Set in the ORB-SLAM2 configuration file.
     */
    double scaleFactor;


    /* *
     * Number of levels of the pyramid.
     * Set in the ORB-SLAM2 configuration file.
     */
    int nlevels;


    /* *
     * Initial FAST threshold, used for detection of single points in all images.
     * Set in the ORB-SLAM2 configuration file.
     */
    int iniThFAST;

    /* *
     * Minimum FAST threshold, to maximize the number of singular points detected in a grid cell, when the initial threshold yields few results.
     * Set in the ORB-SLAM2 configuration file.
     */
    int minThFAST;

    /* *
     * Number of singular points desired by each level of the pyramid.
     * All your values ​​are calculated from nfeatures.
     * Vector length nlevels.
     */
    std::vector<int> mnFeaturesPerLevel;


    /* *
     * Circular patch edge of diameter 31.
     * Umx is a vector of 'HALF_PATCH_SIZE + 1' elements, with the limit of each line in a quarter of a circle.
     * It has 16 elements for a circumference of 31 pixels in diameter, the patch within which the descriptor is extracted.
     * It is calculated in the constructor.
     */
    std::vector<int> umax;

    /* *
     * Scale factor abosluto of each level of the pyramid, calculated from scaleFactor.
     * Vector length nlevels.
     */
    std::vector<float> mvScaleFactor;

    /* *
     * Inverse precalculated mvScaleFactor.
     * Vector length nlevels.
     */
    std::vector<float> mvInvScaleFactor;

    /* *
     * MvScaleFactor square.
     */    
    std::vector<float> mvLevelSigma2;

    /* *
     * Pre-calculated inverse of mvLevelSigma2.
     */
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

