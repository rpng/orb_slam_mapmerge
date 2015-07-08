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

#include "threads/OrbThread.h"

#include <ros/ros.h>

namespace ORB_SLAM
{

    OrbThread::OrbThread(MapDatabase* pMap)
    {
        mapDB = pMap;
        mbResetRequested = false;
        mbStopped = false;
        mbStopRequested = false;
    }
    
    void OrbThread::SetThreads(LocalMapping* pLocalMapper, LoopClosing* pLoopCloser, MapMerging* pMapMerger, Tracking* pTracker)
    {
        mpLocalMapper = pLocalMapper;
        mpLoopCloser = pLoopCloser;
        mpMapMerger = pMapMerger;
        mpTracker = pTracker;
    }
    
    void OrbThread::RequestStop()
    {
        boost::mutex::scoped_lock lock(mMutexStop);
        mbStopRequested = true;
    }

    void OrbThread::RequestReset()
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            mbResetRequested = true;
        }
        ros::Rate r(500);
        while(ros::ok())
        {
            {
            boost::mutex::scoped_lock lock2(mMutexReset);
            if(!mbResetRequested)
                break;
            }
            r.sleep();
        }
    }

    void OrbThread::Stop()
    {
        boost::mutex::scoped_lock lock(mMutexStop);
        mbStopped = true;
    }

    void OrbThread::Release()
    {
        boost::mutex::scoped_lock lock(mMutexStop);
        mbStopped = false;
        mbStopRequested = false;
    }

    bool OrbThread::isStopped()
    {
        boost::mutex::scoped_lock lock(mMutexStop);
        return mbStopped;
    }

    bool OrbThread::stopRequested()
    {
        boost::mutex::scoped_lock lock(mMutexStop);
        return mbStopRequested;
    }
    
    void OrbThread::ResetIfRequested()
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        if(mbResetRequested)
            mbResetRequested=false;
    }

}//namespace ORB_SLAM