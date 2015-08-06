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

#include "publishers/MapPublisher.h"

#include "types/MapPoint.h"
#include "types/KeyFrame.h"

namespace ORB_SLAM
{


MapPublisher::MapPublisher(MapDatabase* pMap):mpMap(pMap), mbCameraUpdated(false)
{
    // Set our key variables
    MAP_FRAME_ID = new string("/ORB_SLAM/World");
    POINTS_NAMESPACE = new string("MapPoints");
    KEYFRAMES_NAMESPACE = new string("KeyFrames");
    GRAPH_NAMESPACE = new string("Graph");
    CAMERA_NAMESPACE = new string("Camera");
    
    //Configure MapPoints
    fPointSize=0.01;
    mPoints_Curr.header.frame_id = MAP_FRAME_ID->c_str();
    mPoints_Curr.ns = POINTS_NAMESPACE->c_str();
    mPoints_Curr.id=1;
    mPoints_Curr.type = visualization_msgs::Marker::POINTS;
    mPoints_Curr.scale.x=fPointSize;
    mPoints_Curr.scale.y=fPointSize;
    mPoints_Curr.pose.orientation.w=1.0;
    mPoints_Curr.action=visualization_msgs::Marker::ADD;
    mPoints_Curr.color.a = 1.0;

    //Configure KeyFrames
    fCameraSize=0.04;
    mKeyFrames_Curr.header.frame_id = MAP_FRAME_ID->c_str();
    mKeyFrames_Curr.ns = KEYFRAMES_NAMESPACE->c_str();
    mKeyFrames_Curr.id=2;
    mKeyFrames_Curr.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames_Curr.scale.x=0.005;
    mKeyFrames_Curr.pose.orientation.w=1.0;
    mKeyFrames_Curr.action=visualization_msgs::Marker::ADD;

    mKeyFrames_Curr.color.b=1.0f;
    mKeyFrames_Curr.color.a = 1.0;

    //Configure Covisibility Graph
    mCovisibilityGraph_Curr.header.frame_id = MAP_FRAME_ID->c_str();
    mCovisibilityGraph_Curr.ns = GRAPH_NAMESPACE->c_str();
    mCovisibilityGraph_Curr.id=3;
    mCovisibilityGraph_Curr.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph_Curr.scale.x=0.002;
    mCovisibilityGraph_Curr.pose.orientation.w=1.0;
    mCovisibilityGraph_Curr.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph_Curr.color.b=0.7f;
    mCovisibilityGraph_Curr.color.g=0.7f;
    mCovisibilityGraph_Curr.color.a = 0.3;

    //Configure KeyFrames Spanning Tree
    mMST_Curr.header.frame_id = MAP_FRAME_ID->c_str();
    mMST_Curr.ns = GRAPH_NAMESPACE->c_str();
    mMST_Curr.id=4;
    mMST_Curr.type = visualization_msgs::Marker::LINE_LIST;
    mMST_Curr.scale.x=0.005;
    mMST_Curr.pose.orientation.w=1.0;
    mMST_Curr.action=visualization_msgs::Marker::ADD;
    mMST_Curr.color.b=0.0f;
    mMST_Curr.color.g=1.0f;
    mMST_Curr.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID->c_str();
    mCurrentCamera.ns = CAMERA_NAMESPACE->c_str();
    mCurrentCamera.id=5;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Reference MapPoints
    mReferencePoints_Curr.header.frame_id = MAP_FRAME_ID->c_str();
    mReferencePoints_Curr.ns = POINTS_NAMESPACE->c_str();
    mReferencePoints_Curr.id=6;
    mReferencePoints_Curr.type = visualization_msgs::Marker::POINTS;
    mReferencePoints_Curr.scale.x=fPointSize;
    mReferencePoints_Curr.scale.y=fPointSize;
    mReferencePoints_Curr.pose.orientation.w=1.0;
    mReferencePoints_Curr.action=visualization_msgs::Marker::ADD;
    mReferencePoints_Curr.color.r =1.0f;
    mReferencePoints_Curr.color.a = 1.0;
    
    // Create our marker array clearer
    mDelete_All.markers.resize(1);
    mDelete_All.markers[0].header.frame_id = MAP_FRAME_ID->c_str();
    mDelete_All.markers[0].ns = GRAPH_NAMESPACE->c_str();
    mDelete_All.markers[0].id=0;
    mDelete_All.markers[0].action=3;

    //Configure publisher
    publisher_cur = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/Map", 10);
    publisher_all = nh.advertise<visualization_msgs::MarkerArray>("ORB_SLAM/Map_Array", 10);

    publisher_cur.publish(mPoints_Curr);
    publisher_cur.publish(mReferencePoints_Curr);
    publisher_cur.publish(mCovisibilityGraph_Curr);
    publisher_cur.publish(mKeyFrames_Curr);
    publisher_cur.publish(mCurrentCamera);
}

void MapPublisher::Reset()
{
}

void MapPublisher::Refresh()
{
    if(isCamUpdated())
    {
       cv::Mat Tcw = GetCurrentCameraPose();
       PublishCurrentCamera(Tcw);
       ResetCamFlag();
    }
    if(mpMap->getCurrent() != NULL && mpMap->getCurrent()->isMapUpdated())
    {
        // Send a clear command
        mDelete_All.markers[0].header.stamp = ros::Time::now();
        publisher_all.publish(mDelete_All);
    
        PublishMapPoints();   
        PublishKeyFrames();
        
        // Update our reset flag if needed
        if(mpMap->getCurrent() != NULL)
            mpMap->getCurrent()->ResetUpdated();
    } 
}

void MapPublisher::PublishMapPoints()
{
    // Clear our old current map
    mPoints_Curr.points.clear();
    mReferencePoints_Curr.points.clear();
    // Clear map array
    mPoints_All.markers.clear();
    mReferencePoints_All.markers.clear();
    // Resize our arrays
    size_t size = mpMap->getAll().size();
    mPoints_All.markers.resize(size);
    mReferencePoints_All.markers.resize(size);
    
    // Loop through each map so we can render
    for(size_t j=0; j < mpMap->getAll().size(); j++)
    {
        // Ensure we are  not erased
        if(mpMap->getAll().at(j)->getErased())
            continue;

        // Get our mappoints for the map
        vector<MapPoint*> vpMPs = mpMap->getMap(j)->GetAllMapPoints();
        vector<MapPoint*> vpRefMPs = mpMap->getMap(j)->GetReferenceMapPoints();
        // Create a set so we can compare counts
        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
        
        // Create namespace
        std::ostringstream oss;
        oss << j+1 << " " << *POINTS_NAMESPACE;
        
        // Configure Keyframe array
        mPoints_All.markers[j].header.frame_id = MAP_FRAME_ID->c_str();
        mPoints_All.markers[j].ns = oss.str().c_str();
        mPoints_All.markers[j].id=j*size+3;
        mPoints_All.markers[j].type = visualization_msgs::Marker::POINTS;
        mPoints_All.markers[j].scale.y=fPointSize;
        mPoints_All.markers[j].scale.x=fPointSize;
        mPoints_All.markers[j].pose.orientation.w=1.0;
        mPoints_All.markers[j].action=visualization_msgs::Marker::ADD;
        mPoints_All.markers[j].color.a = 1.0;
        
        // Configure Keyframe array
        mReferencePoints_All.markers[j].header.frame_id = MAP_FRAME_ID->c_str();
        mReferencePoints_All.markers[j].ns = oss.str().c_str();
        mReferencePoints_All.markers[j].id=j*size+4;
        mReferencePoints_All.markers[j].type = visualization_msgs::Marker::POINTS;
        mReferencePoints_All.markers[j].scale.y=fPointSize;
        mReferencePoints_All.markers[j].scale.x=fPointSize;
        mReferencePoints_All.markers[j].pose.orientation.w=1.0;
        mReferencePoints_All.markers[j].action=visualization_msgs::Marker::ADD;
        mReferencePoints_All.markers[j].color.r = 1.0f;
        mReferencePoints_All.markers[j].color.a = 1.0;
        
        // Set time, created
        mPoints_All.markers[j].header.stamp = ros::Time::now();
        mReferencePoints_All.markers[j].header.stamp = ros::Time::now();

        // Loop through mappoints of current map
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            // Check to make sure the mappoint has not been deleted
            if(!vpMPs[i] || spRefMPs.count(vpMPs[i]))
                continue;
            geometry_msgs::Point p;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);
            
            // Add to our current map
            if(mpMap->getMap(j) == mpMap->getCurrent())
            {
                mPoints_Curr.points.push_back(p);
            }
            // Always add to array
            mPoints_All.markers[j].points.push_back(p);
        }

        for(size_t i=0, iend=vpRefMPs.size(); i<iend;i++)
        {
            // Check to make sure the mappoint has not been deleted
            if(!vpRefMPs[i] || vpRefMPs[i] == NULL)
                continue;
            geometry_msgs::Point p;
            cv::Mat pos = vpRefMPs[i]->GetWorldPos();            
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);

            // Add to our current map
            if(mpMap->getMap(j) == mpMap->getCurrent())
            {
                mReferencePoints_Curr.points.push_back(p);
            }
            // Always add to array
            mReferencePoints_All.markers[j].points.push_back(p);
        }
        
    }
    
    // Set time created
    mPoints_Curr.header.stamp = ros::Time::now();
    mReferencePoints_Curr.header.stamp = ros::Time::now();

    // Publish current
    publisher_cur.publish(mPoints_Curr);
    publisher_cur.publish(mReferencePoints_Curr);
    
    // Publish the map array
    publisher_all.publish(mPoints_All);
    publisher_all.publish(mReferencePoints_All);
}

void MapPublisher::PublishKeyFrames()
{
    // Clear our old current map
    mKeyFrames_Curr.points.clear();
    mCovisibilityGraph_Curr.points.clear();
    mMST_Curr.points.clear();
    // Clear map array
    mKeyFrames_All.markers.clear();
    mCovisibilityGraph_All.markers.clear();
    mMST_All.markers.clear();
    // Resize our arrays
    size_t size = mpMap->getAll().size();
    mKeyFrames_All.markers.resize(size);
    mCovisibilityGraph_All.markers.resize(size);
    mMST_All.markers.resize(size);
    
    // Loop through each map so we can render
    for(size_t j=0; j < mpMap->getAll().size(); j++)
    {
        // Ensure we are erased
        if(mpMap->getAll().at(j)->getErased())
            continue;

        // Get our keyframes for the map
        vector<KeyFrame*> vpKFs = mpMap->getMap(j)->GetAllKeyFrames();

        float d = fCameraSize;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);
        
        // Create namespace
        std::ostringstream oss;
        oss << j+1 << " " << *KEYFRAMES_NAMESPACE;
        
        // Configure Keyframe array
        mKeyFrames_All.markers[j].header.frame_id = MAP_FRAME_ID->c_str();
        mKeyFrames_All.markers[j].ns = oss.str().c_str();
        mKeyFrames_All.markers[j].id=j*size+0;
        mKeyFrames_All.markers[j].type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames_All.markers[j].scale.x=0.005;
        mKeyFrames_All.markers[j].pose.orientation.w=1.0;
        mKeyFrames_All.markers[j].action=visualization_msgs::Marker::ADD;
        mKeyFrames_All.markers[j].color.a = 1.0;
        
        // Create namespace
        oss.clear();
        oss.str("");
        oss << j+1 << " " << *GRAPH_NAMESPACE;
        
        // Configure graph array
        mCovisibilityGraph_All.markers[j].header.frame_id = MAP_FRAME_ID->c_str();
        mCovisibilityGraph_All.markers[j].ns = oss.str().c_str();
        mCovisibilityGraph_All.markers[j].id=j*size+1;
        mCovisibilityGraph_All.markers[j].type = visualization_msgs::Marker::LINE_LIST;
        mCovisibilityGraph_All.markers[j].scale.x=0.002;
        mCovisibilityGraph_All.markers[j].pose.orientation.w=1.0;
        mCovisibilityGraph_All.markers[j].action=visualization_msgs::Marker::ADD;
         mCovisibilityGraph_All.markers[j].color.b=0.7f;
         mCovisibilityGraph_All.markers[j].color.g=0.7f;
         mCovisibilityGraph_All.markers[j].color.a = 0.3;

        // Configure mst array
        mMST_All.markers[j].header.frame_id = MAP_FRAME_ID->c_str();
        mMST_All.markers[j].ns = oss.str().c_str();
        mMST_All.markers[j].id=j*size+2;
        mMST_All.markers[j].type = visualization_msgs::Marker::LINE_LIST;
        mMST_All.markers[j].scale.x=0.005;
        mMST_All.markers[j].pose.orientation.w=1.0;
        mMST_All.markers[j].action=visualization_msgs::Marker::ADD;
        mMST_All.markers[j].color.b=0.0f;
        mMST_All.markers[j].color.g=1.0f;
        mMST_All.markers[j].color.a = 1.0;

        // Set time, created
        mKeyFrames_All.markers[j].header.stamp = ros::Time::now();
        mCovisibilityGraph_All.markers[j].header.stamp = ros::Time::now();
        mMST_All.markers[j].header.stamp = ros::Time::now();

        for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
        {
            // Skip keyframes that are bad
            if(vpKFs[i] == NULL || vpKFs[i]->isBad())
                continue;

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
            
            // Add to our current map
            if(mpMap->getMap(j) == mpMap->getCurrent())
            {
                mKeyFrames_Curr.points.push_back(msgs_o);
                mKeyFrames_Curr.points.push_back(msgs_p1);
                mKeyFrames_Curr.points.push_back(msgs_o);
                mKeyFrames_Curr.points.push_back(msgs_p2);
                mKeyFrames_Curr.points.push_back(msgs_o);
                mKeyFrames_Curr.points.push_back(msgs_p3);
                mKeyFrames_Curr.points.push_back(msgs_o);
                mKeyFrames_Curr.points.push_back(msgs_p4);
                mKeyFrames_Curr.points.push_back(msgs_p1);
                mKeyFrames_Curr.points.push_back(msgs_p2);
                mKeyFrames_Curr.points.push_back(msgs_p2);
                mKeyFrames_Curr.points.push_back(msgs_p3);
                mKeyFrames_Curr.points.push_back(msgs_p3);
                mKeyFrames_Curr.points.push_back(msgs_p4);
                mKeyFrames_Curr.points.push_back(msgs_p4);
                mKeyFrames_Curr.points.push_back(msgs_p1);
            }
            // Always add to the main array
            mKeyFrames_All.markers[j].points.push_back(msgs_o);
            mKeyFrames_All.markers[j].points.push_back(msgs_p1);
            mKeyFrames_All.markers[j].points.push_back(msgs_o);
            mKeyFrames_All.markers[j].points.push_back(msgs_p2);
            mKeyFrames_All.markers[j].points.push_back(msgs_o);
            mKeyFrames_All.markers[j].points.push_back(msgs_p3);
            mKeyFrames_All.markers[j].points.push_back(msgs_o);
            mKeyFrames_All.markers[j].points.push_back(msgs_p4);
            mKeyFrames_All.markers[j].points.push_back(msgs_p1);
            mKeyFrames_All.markers[j].points.push_back(msgs_p2);
            mKeyFrames_All.markers[j].points.push_back(msgs_p2);
            mKeyFrames_All.markers[j].points.push_back(msgs_p3);
            mKeyFrames_All.markers[j].points.push_back(msgs_p3);
            mKeyFrames_All.markers[j].points.push_back(msgs_p4);
            mKeyFrames_All.markers[j].points.push_back(msgs_p4);
            mKeyFrames_All.markers[j].points.push_back(msgs_p1);

            // Covisibility Graph
            vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit) == NULL || (*vit)->isBad())
                        continue;
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    geometry_msgs::Point msgs_o2;
                    msgs_o2.x=Ow2.at<float>(0);
                    msgs_o2.y=Ow2.at<float>(1);
                    msgs_o2.z=Ow2.at<float>(2);
                     // Add to our current map
                    if(mpMap->getAll().at(j) == mpMap->getCurrent())
                    {
                        mCovisibilityGraph_Curr.points.push_back(msgs_o);
                        mCovisibilityGraph_Curr.points.push_back(msgs_o2);
                    }
                    // Always add to our main array
                    mCovisibilityGraph_All.markers[j].points.push_back(msgs_o);
                    mCovisibilityGraph_All.markers[j].points.push_back(msgs_o2);
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
                // Add to our current map
                if(mpMap->getMap(j) == mpMap->getCurrent())
                {
                    mMST_Curr.points.push_back(msgs_o);
                    mMST_Curr.points.push_back(msgs_op);
                }
                // Always add to our main array
                mMST_All.markers[j].points.push_back(msgs_o);
                mMST_All.markers[j].points.push_back(msgs_op);
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
                mMST_Curr.points.push_back(msgs_o);
                mMST_Curr.points.push_back(msgs_ol);
            }
        }
    }

    // Set what time we have updated the current map
    mKeyFrames_Curr.header.stamp = ros::Time::now();
    mCovisibilityGraph_Curr.header.stamp = ros::Time::now();
    mMST_Curr.header.stamp = ros::Time::now();
    
    // Publish the current map
    publisher_cur.publish(mKeyFrames_Curr);
    publisher_cur.publish(mCovisibilityGraph_Curr);
    publisher_cur.publish(mMST_Curr);
    
    // Publish the map array
    publisher_all.publish(mKeyFrames_All);
    publisher_all.publish(mCovisibilityGraph_All);
    publisher_all.publish(mMST_All);
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

    publisher_cur.publish(mCurrentCamera);
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

} //namespace ORB_SLAM
