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

#include "types/KeyFrame.h"
#include "types/Map.h"
#include "types/MapDatabase.h"
#include "types/ORBVocabulary.h"
#include "types/KeyFrameDatabase.h"

#include "threads/OrbThread.h"
#include "threads/LocalMapping.h"
#include "threads/LoopClosing.h"
#include "threads/Tracking.h"

#include <boost/thread.hpp>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class KeyFrame;
class Tracking;
class LocalMapping;
class LoopClosing;

class MapMerging: public OrbThread
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
    
public:
    MapMerging(MapDatabase *mapDB);
    
    void Run();
    
    void InsertKeyFrame(KeyFrame *pKF);
  
protected:

    bool CheckNewKeyFrames();

    bool DetectLoop(Map* map);

    bool ComputeSim3(Map* map);

    void CorrectLoop(Map* map);
    
    void SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap);

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
    g2o::Sim3 mpMatchedgScm;
    g2o::Sim3 mg2oScw;
    double mScale_cw;

    long unsigned int mLastLoopKFid;

};
} //namespace ORB_SLAM

#endif // MAPCLOSING_H