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

#ifndef ORBTHREAD_H
#define ORBTHREAD_H

#include "types/MapDatabase.h"

//#include "threads/Tracking.h"
//#include "threads/LocalMapping.h"
//#include "threads/LoopClosing.h"
//#include "threads/MapMerging.h"

#include <boost/thread.hpp>

namespace ORB_SLAM
{

class Tracking;
class Relocalization;
class LocalMapping;
class LoopClosing;
class MapMerging;
class MapDatabase;

class OrbThread
{
    public:

        OrbThread(MapDatabase* pMap);
        
        // Set the links to other threads
        void SetThreads(LocalMapping* pLocalMapper, LoopClosing* pLoopCloser, MapMerging* pMapMerger, Relocalization* pRelocalizer, Tracking* pTracker);
        
        // Our main run function for the thread
        // The subclass needs to handle this
        virtual void Run() =0;
        
        // Thread Syncing
        void RequestStop();
        void RequestReset();
        void Stop();
        void Release();
        
        // Current start/stop status
        bool isStopped();
        bool stopRequested();

    protected:
    
        // Thread reseting
        virtual void ResetIfRequested();
        boost::mutex mMutexReset;
        bool mbResetRequested;

        // Our map db
        MapDatabase* mapDB;
        
        // Other threads
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopCloser;
        MapMerging* mpMapMerger;
        Relocalization* mpRelocalizer;
        Tracking* mpTracker;
        
        // Thread syncing vars
        boost::mutex mMutexStop;
        bool mbStopped;
        bool mbStopRequested;

};

} //namespace ORB_SLAM

#endif // ORBTHREAD_H
