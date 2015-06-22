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

#include "MapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include <stdlib.h>

namespace ORB_SLAM
{


MapPublisher::MapPublisher(MapDatabase* pMap):mpMap(pMap), mbCameraUpdated(false)
{
    MAP_FRAME_ID = new std::string("/ORB_SLAM/World");
    POINTS_NAMESPACE = new std::string("MapPoints");
    KEYFRAMES_NAMESPACE = new std::string("KeyFrames");
    GRAPH_NAMESPACE = new std::string("Graph");
    CAMERA_NAMESPACE = new std::string("Camera");

    //Configure MapPoints
    fPointSize=0.01;

    //Configure KeyFrames
    fCameraSize=0.02;
    mKeyFrames.header.frame_id = MAP_FRAME_ID->c_str();
    mKeyFrames.ns = KEYFRAMES_NAMESPACE->c_str();
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;

    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;

    //Configure Covisibility Graph
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID->c_str();
    mCovisibilityGraph.ns = GRAPH_NAMESPACE->c_str();
    mCovisibilityGraph.id=2;
    mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph.scale.x=0.002;
    mCovisibilityGraph.pose.orientation.w=1.0;
    mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph.color.b=0.7f;
    mCovisibilityGraph.color.g=0.7f;
    mCovisibilityGraph.color.a = 0.3;

    //Configure KeyFrames Spanning Tree
    mMST.header.frame_id = MAP_FRAME_ID->c_str();
    mMST.ns = GRAPH_NAMESPACE->c_str();
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID->c_str();
    mCurrentCamera.ns = CAMERA_NAMESPACE->c_str();
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Publisher
    publisher_map = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/Map", 1000);
    publisher_arr = nh.advertise<visualization_msgs::MarkerArray>("ORB_SLAM/Map_Array", 1000);

    publisher_arr.publish(mPoints);
    publisher_arr.publish(mReferencePoints);
    publisher_map.publish(mCovisibilityGraph);
    publisher_map.publish(mKeyFrames);
    publisher_map.publish(mCurrentCamera);
}

void MapPublisher::Refresh()
{
    if(isCamUpdated())
    {
       cv::Mat Tcw = GetCurrentCameraPose();
       PublishCurrentCamera(Tcw);
       ResetCamFlag();
    }
    if(mpMap->getLatestMap() != NULL && mpMap->getLatestMap()->isMapUpdated())
    { 
        // Marker array size
        int size = mpMap->getAllMaps().size();
        // Clear old points
        mPoints.markers.clear();
        mPoints.markers.resize(size);
        mReferencePoints.markers.clear();
        mReferencePoints.markers.resize(size);
        // Clear old key frames
        mKeyFrames.points.clear();
        mCovisibilityGraph.points.clear();
        mMST.points.clear();
        
        // Loop through all maps
        for(size_t i=0, iend =  size; i<iend; i++) {
          
            vector<KeyFrame*> vKeyFrames = mpMap->getAllMaps().at(i)->GetAllKeyFrames();
            vector<MapPoint*> vMapPoints = mpMap->getAllMaps().at(i)->GetAllMapPoints();
            vector<MapPoint*> vRefMapPoints = mpMap->getAllMaps().at(i)->GetReferenceMapPoints();

            PublishMapPoints(vMapPoints, vRefMapPoints, size, i);   
            PublishKeyFrames(vKeyFrames);
            
            mpMap->getAllMaps().at(i)->ResetUpdated();
        }
        
        // Publish points
        publisher_arr.publish(mPoints);
        publisher_arr.publish(mReferencePoints);
        // Publish Keyframes
        publisher_map.publish(mKeyFrames);
        publisher_map.publish(mCovisibilityGraph);
        publisher_map.publish(mMST);
    }    
}

void MapPublisher::PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs, int size, int ct)
{ 
    // Create namespace
    std::ostringstream oss;
    oss << ct << "_" << *POINTS_NAMESPACE;
    std::string ns = oss.str();
    // Map point life time
    ros::Duration lifetime = ros::Duration(0,20);
    
    // Configure map points
    mPoints.markers[ct].header.frame_id = MAP_FRAME_ID->c_str();
    mPoints.markers[ct].ns = ns.c_str();
    mPoints.markers[ct].id=ct;
    mPoints.markers[ct].type = visualization_msgs::Marker::POINTS;
    mPoints.markers[ct].scale.x=fPointSize;
    mPoints.markers[ct].scale.y=fPointSize; 
    mPoints.markers[ct].lifetime= lifetime;
    mPoints.markers[ct].pose.orientation.w=1.0;
    mPoints.markers[ct].action=visualization_msgs::Marker::ADD;
    
    // Configure referance points
    mReferencePoints.markers[ct].header.frame_id = MAP_FRAME_ID->c_str();
    mReferencePoints.markers[ct].ns = ns.c_str();
    mReferencePoints.markers[ct].id=ct+size;
    mReferencePoints.markers[ct].type = visualization_msgs::Marker::POINTS;
    mReferencePoints.markers[ct].scale.x=fPointSize;
    mReferencePoints.markers[ct].scale.y=fPointSize;
    mPoints.markers[ct].lifetime= lifetime;
    mReferencePoints.markers[ct].pose.orientation.w=1.0;
    mReferencePoints.markers[ct].action=visualization_msgs::Marker::ADD;
    

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    
    //std::cout << "Adding: " << inttohash(ct) << " ct: " << ct <<std::endl;

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mPoints.markers[ct].points.push_back(p);
    }

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mReferencePoints.markers[ct].points.push_back(p);
    }
    
    // Set color
    mPoints.markers[ct].color.r = (float)inttohash(ct+1);
    mPoints.markers[ct].color.g =  (float)inttohash(ct+2);
    mPoints.markers[ct].color.b =  (float)inttohash(ct+3);
    mPoints.markers[ct].color.a =  1.0f;
    // Set color
    mReferencePoints.markers[ct].color.r = (float)inttohash(ct+4);
    mReferencePoints.markers[ct].color.g =  (float)inttohash(ct+5);
    mReferencePoints.markers[ct].color.b =  (float)inttohash(ct+6);
    mReferencePoints.markers[ct].color.a =  1.0f;
    // Set time
    mPoints.markers[ct].header.stamp = ros::Time::now();
    mReferencePoints.markers[ct].header.stamp = ros::Time::now();
}

void MapPublisher::PublishKeyFrames(const vector<KeyFrame*> &vpKFs)
{
    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
    {
        cv::Mat Tcw = vpKFs[i]->GetPose();
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow.at<float>(0);
        msgs_o.y=ow.at<float>(1);
        msgs_o.z=ow.at<float>(2);
        msgs_p1.x=p1w.at<float>(0);
        msgs_p1.y=p1w.at<float>(1);
        msgs_p1.z=p1w.at<float>(2);
        msgs_p2.x=p2w.at<float>(0);
        msgs_p2.y=p2w.at<float>(1);
        msgs_p2.z=p2w.at<float>(2);
        msgs_p3.x=p3w.at<float>(0);
        msgs_p3.y=p3w.at<float>(1);
        msgs_p3.z=p3w.at<float>(2);
        msgs_p4.x=p4w.at<float>(0);
        msgs_p4.y=p4w.at<float>(1);
        msgs_p4.z=p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        // Covisibility Graph
        vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                geometry_msgs::Point msgs_o2;
                msgs_o2.x=Ow2.at<float>(0);
                msgs_o2.y=Ow2.at<float>(1);
                msgs_o2.z=Ow2.at<float>(2);
                mCovisibilityGraph.points.push_back(msgs_o);
                mCovisibilityGraph.points.push_back(msgs_o2);
            }
        }

        // MST
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(0);
            msgs_op.y=Owp.at<float>(1);
            msgs_op.z=Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            geometry_msgs::Point msgs_ol;
            msgs_ol.x=Owl.at<float>(0);
            msgs_ol.y=Owl.at<float>(1);
            msgs_ol.z=Owl.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_ol);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
    mCurrentCamera.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

    publisher_map.publish(mCurrentCamera);
}

void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

cv::Mat MapPublisher::GetCurrentCameraPose()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    return mCameraPose.clone();
}

bool MapPublisher::isCamUpdated()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mbCameraUpdated = false;
}

double MapPublisher::inttohash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return abs(key / 2147483647.5 - 1);
}

} //namespace ORB_SLAM
