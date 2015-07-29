/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "types/Map.h"
#include "types/MapDatabase.h"
#include "types/MapPoint.h"
#include "types/KeyFrame.h"

namespace ORB_SLAM
{

class MapPublisher
{
public:
    MapPublisher(MapDatabase* pMap);

    MapDatabase* mpMap;

    void Refresh();
    void PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs);
    void PublishKeyFrames();
    void PublishCurrentCamera(const cv::Mat &Tcw);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher_cur;
    ros::Publisher publisher_all;

    // Camera
    visualization_msgs::Marker mCurrentCamera;
    
    // Current map
    visualization_msgs::Marker mPoints_Curr;
    visualization_msgs::Marker mReferencePoints_Curr;
    // Current keyframes
    visualization_msgs::Marker mKeyFrames_Curr;
    visualization_msgs::Marker mCovisibilityGraph_Curr;
    visualization_msgs::Marker mMST_Curr;
    
    // All maps
    visualization_msgs::MarkerArray mPoints_All;
    visualization_msgs::MarkerArray mReferencePoints_All;
    // All keyframes
    visualization_msgs::MarkerArray mKeyFrames_All;
    visualization_msgs::MarkerArray mCovisibilityGraph_All;
    visualization_msgs::MarkerArray mMST_All;

    float fCameraSize;
    float fPointSize;

    cv::Mat mCameraPose;
    bool mbCameraUpdated;

    boost::mutex mMutexCamera;
    
    string* MAP_FRAME_ID;
    string* POINTS_NAMESPACE;
    string* KEYFRAMES_NAMESPACE;
    string* GRAPH_NAMESPACE;
    string* CAMERA_NAMESPACE;

};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
