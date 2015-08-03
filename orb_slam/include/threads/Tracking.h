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

#ifndef TRACKING_H
#define TRACKING_H

#include "publishers/FramePublisher.h"
#include "publishers/MapPublisher.h"

#include "types/Map.h"
#include "types/MapDatabase.h"
#include "types/Frame.h"
#include "types/ORBVocabulary.h"
#include "types/KeyFrameDatabase.h"

#include "threads/OrbThread.h"
#include "threads/LocalMapping.h"
#include "threads/LoopClosing.h"
#include "threads/MapMerging.h"
#include "threads/Relocalization.h"

#include "util/ORBextractor.h"
#include "util/Initializer.h"
#include "util/FpsCounter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>


namespace ORB_SLAM
{

class FramePublisher;
class MapDatabase;
class Map;
class LocalMapping;
class LoopClosing;
class MapMerging;
class Relocalization;

class Tracking: public OrbThread
{  

public:
    Tracking(FramePublisher* pFramePublisher, MapPublisher* pMapPublisher, MapDatabase* pMap, FpsCounter* pfps, string strSettingPath);

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        INITIALIZING=2,
        WORKING=3
    };

    // This is the main function of the Tracking Thread
    void Run();

    void ForceRelocalisation();
    void ForceInlineRelocalisation();

    eTrackingState mState;
    eTrackingState mLastProcessedState;
        
    // Frames, these should only be accessed by the frame pub class
    Frame mCurrentFrame;
    Frame mLastFrame;

    // Initialization Variables
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    bool publisherStopRequested();
    void publishersStop(bool state);

    bool RelocalisationRequested();
    bool RelocalisationInlineRequested();
    void ResetRelocalisationRequested();
    void SetRelocalisationFrame(Frame* frame);

protected:
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void FirstInitialization();
    void Initialize();
    void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);

    void Reset();
    bool RelocalisationInline();

    bool TrackPreviousFrame();
    bool TrackWithMotionModel();

    void UpdateReference();
    void UpdateReferencePoints();
    void UpdateReferenceKeyFrames();

    bool TrackLocalMap();
    void SearchReferencePointsInFrustum();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();
    
    void UpdateCurrentFrameId();

    //ORB
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;

    // Initalization
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    //Publishers
    FramePublisher* mpFramePublisher;
    MapPublisher* mpMapPublisher;

    //Local Map for init
    Map* localMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    boost::mutex mMutexRelocFrameId;

    //Mutex
    boost::mutex mMutexTrack;
    boost::mutex mMutexForceRelocalisation;
    boost::mutex mMutexForceRelocalisationInline;

    //Reset
    bool mbPublisherStopped;
    bool mbReseting;
    boost::mutex mMutexReset;

    //Is relocalisation requested by an external thread? (loop closing)
    bool mbForceRelocalisation;
    bool mbForceRelocalisationInline;

    //Motion Model
    bool mbMotionModel;
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    // Transfor broadcaster (for visualization in rviz)
    tf::TransformBroadcaster mTfBr;
    
    // Our fps counter
    FpsCounter* fps_counter;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
