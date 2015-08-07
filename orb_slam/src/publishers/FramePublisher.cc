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

#include "publishers/FramePublisher.h"
#include "threads/Tracking.h"
#include "util/FpsCounter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

namespace ORB_SLAM
{

FramePublisher::FramePublisher(FpsCounter* pfps)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mbUpdated = true;
    
    fps_counter = pfps;
    
    mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM/Frame",10,true);

    PublishFrame();
}

void FramePublisher::SetMapDB(MapDatabase *pMap)
{
    mpMap = pMap;
}

void FramePublisher::Refresh()
{
    if(mbUpdated)
    {
        PublishFrame();
        mbUpdated = false;
    }
}

void FramePublisher::Reset()
{
    boost::mutex::scoped_lock lock(mMutex);
    mvCurrentKeys.clear();
    mvbOutliers.clear();
    mvpMatchedMapPoints.clear();
    lastFrameMapPoints.clear();
    mvIniKeys.clear();
    mvIniMatches.clear();
}

cv::Mat FramePublisher::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondences with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    
    vector<MapPoint*> vMatchedMapPoints; // Tracked MapPoints in current frame
    
    int state; // Tracking state
    
    //Copy variable to be used within scoped mutex
    {
        boost::mutex::scoped_lock lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {            
            vIniKeys = mvIniKeys;
        }
        else if(mState==Tracking::INITIALIZING)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::WORKING)
        {
            vCurrentKeys = mvCurrentKeys;
            vMatchedMapPoints = mvpMatchedMapPoints;
        }
    } // destroy scoped mutex -> release

    if(im.channels()<3)
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::INITIALIZING) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::WORKING) //TRACKING
    {
        mnTracked=0;
        const float r = 5;

        // Loop through each mappoint to display matching orb 2D point
        for(unsigned int i=0;i<vMatchedMapPoints.size();i++)
        {
            // Ensure we are a valid mappoint
            if(vMatchedMapPoints[i]==NULL || vMatchedMapPoints[i]->isBad())
                continue;
            // If not an outlier, display
            if(!mvbOutliers[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                // Draw
                cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                mnTracked++;
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

void FramePublisher::PublishFrame()
{
    cv::Mat im = DrawFrame();
    cv_bridge::CvImage rosImage;
    rosImage.image = im.clone();
    rosImage.header.stamp = ros::Time::now();
    rosImage.encoding = "bgr8";

    mImagePub.publish(rosImage.toImageMsg());
    ros::spinOnce();
}

void FramePublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s1;
    stringstream s2;
    if(nState==Tracking::NO_IMAGES_YET)
    {
        s1 << "Fps: " << fps_counter->get();
        s2 << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
    }
    else if(nState==Tracking::NOT_INITIALIZED)
    {
        s1 << "Fps: " << fps_counter->get();
        s2 << "NOT INITIALIZED ";
    }
    else if(nState==Tracking::INITIALIZING)
    {
        s1 << "Fps: " << fps_counter->get();
        s2 << "TRYING TO INITIALIZE ";
    }
    else if(nState==Tracking::WORKING)
    {
        int nKFs=0, nMPs=0, size_cu=0, size_all=0, id=0;
        if(mpMap->getCurrent() != NULL) {
            nKFs = mpMap->getCurrent()->KeyFramesInMap();
            nMPs = mpMap->getCurrent()->MapPointsInMap();
            id = mpMap->getCurrentID();
            // Get total number of maps
            size_all = mpMap->getAll().size();
            // Count how many non-erased maps
            for(size_t j=0; j < mpMap->getAll().size(); j++)
                if(!mpMap->getAll().at(j)->getErased())
                    size_cu++;
        }
        s1 << "Fps: " << fps_counter->get() << " , Maps: " << size_cu << " (" << size_all << ") , MapID: " << id;
        s2 << "KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s1 << "Fps: " << fps_counter->get();
        s2 << "LOADING ORB VOCABULARY.";
    }

    int baseline=0;
    int fontscale=2;
    cv::Size textSize = cv::getTextSize(s2.str(),cv::FONT_HERSHEY_PLAIN,fontscale,1,&baseline);

    // Create our text and black bottom bar
    imText = cv::Mat(im.rows+3*textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(3*textSize.height+10,im.cols,im.type());

    // Display text
    cv::putText(imText,s1.str(),cv::Point(5,imText.rows-2*textSize.height),cv::FONT_HERSHEY_PLAIN,fontscale,cv::Scalar(255,255,255),1,8);
    cv::putText(imText,s2.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,fontscale,cv::Scalar(255,255,255),1,8);

}

void FramePublisher::Update(Tracking *pTracker)
{
    boost::mutex::scoped_lock lock(mMutex);
    pTracker->mCurrentFrame.im.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvpMatchedMapPoints=pTracker->mCurrentFrame.mvpMapPoints;
    mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;

    if(pTracker->mLastProcessedState==Tracking::INITIALIZING)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }

    mState=static_cast<int>(pTracker->mLastProcessedState);

    mbUpdated=true;
}

} //namespace ORB_SLAM
