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

#include "MapClosing.h"

#include <ros/ros.h>

namespace ORB_SLAM
{

MapClosing::MapClosing(MapDatabase *pMap) {
    mapDB = pMap;
}

void MapClosing::SetTracker(Tracking* pTracker) {
    mpTracker=pTracker;
}

void MapClosing::SetLocalMapper(LocalMapping* pLocalMapper) {
    mpLocalMapper=pLocalMapper;
}

 void MapClosing::SetLoopCloser(LoopClosing* pLoopCloser) {
     mpLoopCloser=pLoopCloser;
 }

void MapClosing::Run()
{

    ros::Rate r(1000000);

    while(ros::ok())
    {
        // Check that we have a map initialized
        if(mapDB->getCurrent() != NULL)
        {
            
        }

        r.sleep();
    }
}


} //namespace ORB_SLAM