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

#ifndef RELOCALIZATION
#define RELOCALIZATION

#include "threads/OrbThread.h"
#include "threads/Tracking.h"

#include "types/Frame.h"
#include "types/Map.h"
#include "types/MapDatabase.h"

#include <boost/thread.hpp>

namespace ORB_SLAM
{
    
class Tracking;
class MapDatabase;

class Relocalization: public OrbThread
{
    public:
    
        Relocalization(MapDatabase *mapDB);

        void Run();
        
        bool isAcceptingFrames();
        void AddFrame(Frame* newFrame);
        
        bool relocalizeIfSuccessfull();

        void ResetIfRequested();
    
    protected:
    
        void Relocalisation();
        
        bool isSuccess();
        
        void setAcceptingFrames(bool val);
        
        boost::mutex mMutexAcceptFrames;
        bool acceptingFrames;
        
        boost::mutex mMutexFrame;
        Frame* mCurrentFrame;

        boost::mutex mMutexSuccessCheck;
        bool isSuccessfull;
        Map* mapMatch;
        
};

}

#endif // RELOCALIZATION
