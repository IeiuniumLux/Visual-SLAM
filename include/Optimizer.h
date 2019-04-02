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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

/* *
 * Optimizer concentrates all operations with g2o.
 * This class has no properties, but only a set of static or class methods.
 * Optimizer is not instantiated, it functions as a namespace.
 * Assemble the six functions implemented with framwork g2o, which include bundle adjustment, pose optimization and graph optimization.
 *
 */
class Optimizer
{
public:

  /* *
   * Bundle adjusment over keyframes and map points passed as arguments.
   * Take all keyframes and all map points, to run a BA.
   * @param vpKFs Vector of keyframes. Each keyframe contains the 2d points displayed, and their relationship to the map points.
   * @param vpMP Vector map points.
   * @param nIterations Number of maximum iterations for BA.
   * @param pbStopFlag Signal to force the optimizer to stop.
     * @param nLoopKF
     * @param bRobust Signal that requests a robust evaluator (which supports outliers) rather than a strict one (which assumes all points are valid).
   *
   * This method is invoked only from GlobalBundleAdjustment.
   *
   */
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);


    /* *
     * Run BundleAdjstment by taking map data.
     * Take all keyframes and all map points, to run a BA.
     * @param pMap Map, from where to take all keyframes and points.
     * @param nIterations Number of maximum iterations for the BA, passed as is to BundleAdjustment.
     * @param pbStopFlag Signal to force the optimizer to stop, passed as is to BundleAdjustment.
     * @param nLoopKF
     * @param bRobust Signal that requests a robust evaluator (which supports outliers) rather than a strict one (which assumes all points are valid).
     *
     * This method is only invoked from Tracking :: createInitialMap at the start of the tracking, when the map is small.
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);


      /* * Bundle local adjusment from a keyframe.
     * @param pKF Initial Keyframe, Usually Tracking :: mpCurrentKeyFrame.
     * @param pbStopFlag Signal to force the optimizer to stop.
     *
     * The local BA takes the reference keyframe (usually the current one).
     * From it forms a vector of keyframes covisible with GetVectorCovisibleKeyFrames (),
     * And a vector of map points viewed by them. These keyframes and map points will be modified by the BA.
     * Finally create a vector keyframes fixed (not affected by the BA), with the other keyframes that also observe those points.
     * With this data execute a BA using g2o.
     *
     * This is the periodic tracking bundle.
     *
     * This method is invoked only from LocalMapping :: Run ().
     *
     * The optimizer is armed like this:
     *
        Typedef BlockSolver <BlockSolverTraits <Eigen :: Dynamic, Eigen :: Dynamic>> BlockSolverX;
          G2o :: SparseOptimizer optimizer;
          Optimizer.setAlgorithm (
            New g2o :: OptimizationAlgorithmLevenberg (
              New g2o :: BlockSolverX (
                New g2o :: LinearSolverEigen <g2o :: BlockSolverX :: PoseMatrixType> ()
              )
            )
          );
     * PoseMatrixType is MatrixXD, ie double elements and dimensions to be defined.
     * This code means: optimizer spacing, with algorithm LM, block solver for n-dimensional poses
     * And points of m dimensions, using Cholesky spacing from the Eigen library.
     *
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);



    /* *
     * Compute the pose of a picture.
     *
     * @param pFrame
     */
    /* *
     * Calculate the frame's pose from the observed points and their positions on the map.
     * The pose is a 4x4 roto-mapping matrix in homogeneous coordinates.
     * Uses the pFrame-> mTcw table pose as the initial estimate, usually estimated by the movement model, although it can also be retrieved by relocation.
     * This is the only method that calculates the final pose of a frame. Invoked for each frame.
     * The poses of the pictures are established mainly by this method, although also
     * Other methods (Tracking) initialize pose, copy or estimate by motion model.
     *
     * @param pFrame Box with machetes against map points, whose pose you want to calculate. The pose is stored in pFrame-> mTcw. OrbSlam is always the current frame.
     * @returns The number of optimized correspondences (surviving macheos, inliers).
     *
     * The optimizer is armed like this, identical to the LocalBundleAdjustment:
     *
        G2o :: SparseOptimizer optimizer;
        Optimizer.setAlgorithm (
          New g2o :: OptimizationAlgorithmLevenberg (
            New g2o :: BlockSolverX (
              New g2o :: LinearSolverDense <g2o :: BlockSolverX :: PoseMatrixType> ()
            )
          )
        );
     */
    int static PoseOptimization(Frame* pFrame);

    /* * Optimizes the essential graph to close a loop.
     * When detecting a loop closure, the ends are joined.
     * This method performs a "pose graph optimization"; Executes a BA to distribute the mean square error along the essential graph.
     * @param pMap Full Map.
     * @param pLoopKF Keyframe of the forward end of the loop.
     * @param pCurKF Current keyframe, from the back end of the loop.
     * @param Scurw Sim3 transformation required to adapt the back end to the previous one. Not used.
     * @param NonCorrectedSim3 KeyframeAndPose of current and neighboring keyframes, sim3 unprocessed version.
     * @param CorrectedSim3 KeyframeAndPose of neighboring keyframes, version corrected by sim3.
     * @param LoopConnections Set of keyframes connected thanks to loop closure.
     *
     * This method is invoked exclusively in the final stage of LoopClosing :: CorrectLoop ().
     *
     * Loads in the optimizer all keyframes of the map, marking as fixed only to the one of the previous end of the loop.
     * Loads all map axes: the connections between keyframes, adding the new loop connections reported in loopConnections.
     * Execute 20 iterations, and overturn the result to keyframes poses and points positions, recomputing normal and depth.
     *
     *
     * The optimizer is armed like this:
     *
        Typedef BlockSolver <BlockSolverTraits <7, 3>> BlockSolver_7_3;
        G2o :: SparseOptimizer optimizer;
        G2o :: OptimizationAlgorithmLevenberg * solver = new g2o :: OptimizationAlgorithmLevenberg (
          New g2o :: BlockSolver_7_3 (
            New g2o :: LinearSolverEigen <g2o :: BlockSolver_7_3 :: PoseMatrixType> ()
          )
        );
        Solver-> setUserLambdaInit (1e-16);
        Optimizer.setAlgorithm (solver);
     * PoseMatrixType means that for the Hessian of points we use double matrices of 3x3, and for the Hessian of matrices double of 7x7.
     * This code means: optimizer spacing, with algorithm LM, block solver for 7-dimensional poses
     * And 3-dimensional points, using Cholesky spacing from the Eigen library.
     *
     * Unlike PoseOptimization and LocalBundleAdjustment, a BlockSolver_7_3 is used instead of BlockSolverX, and the LambdaInit is set.
     *
     */
    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);


    /* * Determines the sim3 transformation between two keyframes, which best explains a match.
     * Given two keyframes that presumably observe the same area and are candidates for loop closure,
     * And given a series of matched points observed by both keyframes,
     * OptimizeSim3 () computes the similarity transformation sim3 that matches both views and allows closing the loop.
     *
     * This method is invoked exclusively from LoopClosing :: ComputeSim3 ().
     *
     * The optimizer is armed like this, identical to the LocalBundleAdjustment:
     *
        G2o :: SparseOptimizer optimizer;
        Optimizer.setAlgorithm (
          New g2o :: OptimizationAlgorithmLevenberg (
            New g2o :: BlockSolverX (
              New g2o :: LinearSolverEigen <g2o :: BlockSolverX :: PoseMatrixType> ()
            )
          )
        );
     *
     */
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
