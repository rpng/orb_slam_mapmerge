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

#include "MapDatabase.h"

#include "ros/ros.h"

namespace ORB_SLAM
{

MapDatabase::MapDatabase() {
  // Init varibles
  currentMap = NULL;
  reset = false;
  maps = std::vector<Map*>();
}

void MapDatabase::addMap(Map* map) {
  maps.push_back(map);
  currentMap = map;
}

void MapDatabase::eraseMap(Map* map){
    
}

bool MapDatabase::setMap(Map* m){
    for (std::size_t i = 0; i != maps.size(); ++i) {
        if(maps[i] == m) {
            currentMap = maps[i];
            return true;
        }
    }
    return false;
}

Map* MapDatabase::getLatestMap() {
    return currentMap;
}

void MapDatabase::setReset(bool res) {
    reset = res;
}

bool MapDatabase::isReset() {
    return reset;
}

std::vector<Map*> MapDatabase::getAllMaps() {
  return maps;
}


// std::vector<Map*> MapDatabase::DetectConnectionCandidates(KeyFrame* pKF, float minScore) {
// }

// std::vector<Map*> MapDatabase::DetectRelocalisationCandidates(Frame* F) {
// }

} //namespace ORB_SLAM