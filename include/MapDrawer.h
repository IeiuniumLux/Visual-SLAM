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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

/* *
 * This class contains methods for drawing in pangolin.
 * Draw the camera, keyframes and 3D points on the map.
 * Do not draw directly on Pangolin, but use glew.
 * Viewer :: Run invokes these methods, and overturns the drawn to Pangolin.
 *
 */
class MapDrawer
{
public:

	/* *
     * Constructor that takes its parameters directly from the configuration file.
     *
     * @param pMap World map.
     * @param strSettingPath Path of the configuration file.
     *
     * Configuration is limited to graphic aspects: stroke widths, camera sizes and keyframes, expressed in pixels, for display.
     * The parameters are:
     *
     * - Viewer.KeyFrameSize: size of keyframe marks
     * - Viewer.KeyFrameLineWidth: width of lines when drawing keyframes
     * - Viewer.GraphLineWidth: width of the lines of the graph of keyframes
     * - Viewer.PointSize: size of map points
     * - Viewer.CameraSize: size of the camera drawing
     * - Viewer.CameraLineWidth: line width of the camera
     *
     *
     * Summoned only from the System builder.
     */
    MapDrawer(Map* pMap, const string &strSettingPath);

    /* * World map. */
    Map* mpMap;

    /* *
     * Draws all the 3D points on the map.
     * Dibua all in black, and then in red reference ones (those visualized by the camera).
     * Summoned at each Viewer :: Run iteration.
     *
     * @param color true to see the color map points.
     *
     * Summoned only from Viewer :: Run, on each iteration.
     */
    void DrawMapPoints();

    /* *
     * Draw keyframes and graph on screen.
     *
     * @param bDrawKF true to draw keyframes.
     * @param bDrawGraph true to draw the graph.
     *
     * Summoned only from Viewer :: Run, on each iteration.
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    /* *
     * Draw the camera in the given pose.
     *
     * @param Twc Pose the camera.
     *
     * Summoned only from Viewer :: Run, on each iteration.
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    /* *
     * Reports the camera pose to the MapDrawer object, which copies it to MapDrawer :: mCameraPose.
     *
     * @param Tcw Pose in the world, matrix of roto-mapping in homogeneous coordinates.
     *
     * Summoned only from Tracking :: Track and Tracking :: CreateInitialMapMonocular.
     */
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    /* * Method declared and not defined. */
    void SetReferenceKeyFrame(KeyFrame *pKF);

    /* *
     * Informs Twc, the current pose of the world regarding the camera, in pangolin format.
     *
     * Calculates the current inverted pose in the paonglin :: OpenGlMatrix format
     *
     * @param M Matrix of Twc pose converted to opengl array.
     *
     * The camera pose is conveniently updated in the mCameraPose property.
     *
     * Summoned only from Viewer :: Run.
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:
    /* * Size of the keyframe to draw, defined in the configuration file. */
    float mKeyFrameSize;

    /* * Stroke width for drawing keyframes, defined in the configuration file. */
    float mKeyFrameLineWidth;

    /* * Width of the graph trace, defined in the configuration file. */
    float mGraphLineWidth;

    /* * Size of the map points, defined in the configuration file. */
    float mPointSize;

    /* * Size of the camera to draw, defined in the configuration file. */
    float mCameraSize;

    /* * Stroke width to draw the camera, defined in the configuration file. */
    float mCameraLineWidth;

    /* *
     * Pose the camera.
     *
     * Permanently updated with MapDrawer :: SetCurrentCameraPose invoked by Viewer :: Run.
     */
    cv::Mat mCameraPose;

    /* *
     * Mutex that prevents changes in the pose of the camera when you are extracting the vectors rotation and translation.
     */

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
