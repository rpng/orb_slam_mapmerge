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

#include "MapPoint.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Frame.h"

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
  MapDatabase();

  // Adds a new map to the database
  void addMap(Map* map);

  // Removes a map from the database
  void eraseMap(Map* map);

  // Gets the current map that we are tracking
  Map* getLatestMap();
  
    // Gets all maps
  std::vector<Map*> getAllMaps();

  // Map Connection Detection
  // std::vector<Map*> DetectConnectionCandidates(KeyFrame* pKF, float minScore);

  // Map Relocalisation Detection
  // std::vector<Map*> DetectRelocalisationCandidates(Frame* F);

protected:

  // Current map
  Map* currentMap;

  // List of all maps we have
  std::vector<Map*> maps;

  // Mutex
  boost::mutex mapMutex;

};

} //namespace ORB_SLAM

#endif // MAPDATABASE_H