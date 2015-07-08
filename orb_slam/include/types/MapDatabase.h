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

#ifndef MAPDATABASE_H
#define MAPDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "types/MapPoint.h"
#include "types/Map.h"
#include "types/KeyFrame.h"
#include "types/Frame.h"
#include "types/ORBVocabulary.h"

#include <boost/thread.hpp>

namespace ORB_SLAM
{

class MapPoint;
class Map;
class KeyFrame;
class Frame;

class MapDatabase
{
public:

    // Constructor
    MapDatabase(ORBVocabulary* vocab);
    
    // Returns a complete map object
    // This will be loaded with the vocab we have
    Map* getNewMap();
    
    // Adds a new map to the database
    void addMap(Map* map);

    // Removes a map from the database
    void eraseMap(Map* map);
    
    // Set map from the database
    bool setMap(Map* map);
    
    // Gets the current map that we are tracking
    Map* getCurrent();
    
    // Gets current map id
    int getCurrentID();
    
    // Gets the vocab object
    ORBVocabulary* getVocab();

    // Gets all maps
    std::vector<Map*> getAll();
    
    // Returns the older of the two maps
    Map* getOldest(Map* m1, Map* m2);

protected:

    // The ID of the current map
    unsigned int currentMapID;
    
    // Vocabulary
    ORBVocabulary* vocab;

    // List of all maps we have
    std::vector<Map*> maps;

    // Mutex
    boost::mutex mapMutex;
    boost::mutex vocMutex;

};

} //namespace ORB_SLAM

#endif // MAPDATABASE_H