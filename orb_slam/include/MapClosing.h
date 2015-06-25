/**
* This file is part of ORB-SLAM.
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

#ifndef MAPCLOSING_H
#define MAPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDatabase.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include <boost/thread.hpp>

#include "KeyFrameDatabase.h"

#include <g2o/types/sim3/types_seven_dof_expmap.h>


namespace ORB_SLAM
{
    
class KeyFrame;
class LocalMapping;
class LoopClosing;

class MapClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
    
public:
    MapClosing(MapDatabase *mapDB);
    
    void Run();
    
    void InsertKeyFrame(KeyFrame *pKF);
    
    void SetTracker(Tracking* pTracker);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopCloser(LoopClosing* pLoopCloser);
    
    void gracefullStart();
    void gracefullStop();    
  
protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();
    
    MapDatabase* mapDB;

    Tracking* mpTracker;
    LocalMapping *mpLocalMapper;
    LoopClosing* mpLoopCloser;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;
    boost::mutex mMutexLoopQueue;
    
    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;
    double mScale_cw;

    long unsigned int mLastLoopKFid;
    
    bool gracefullStatus;

};
} //namespace ORB_SLAM

#endif // MAPCLOSING_H