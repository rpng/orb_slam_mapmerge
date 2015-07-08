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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "MapDatabase.h"
#include "Map.h"
#include "LoopClosing.h"
#include "MapMerging.h"
#include "Tracking.h"
#include <boost/thread.hpp>
#include "KeyFrameDatabase.h"


namespace ORB_SLAM
{

class Tracking;
class LoopClosing;
class MapMerging;
class MapDatabase;
class Map;

class LocalMapping
{
public:
    LocalMapping(MapDatabase* pMap);

    void SetLoopCloser(LoopClosing* pLoopCloser);
    
    void SetMapMerger(MapMerging* pMapMerger);

    void SetTracker(Tracking* pTracker);

    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();

    void Stop();

    void Release();

    bool isStopped();

    bool stopRequested();

    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);

    void InterruptBA();
    
    void gracefullStart();
    void gracefullStop();    

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;
    boost::mutex mMutexReset;

    MapDatabase* mpMap;

    LoopClosing* mpLoopCloser;
    MapMerging* mpMapMerger;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    boost::mutex mMutexNewKFs;    

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    boost::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    boost::mutex mMutexAccept;
    
    bool gracefullStatus;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
