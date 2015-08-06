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

#include "types/MapDatabase.h"

#include <ros/ros.h>
#include <algorithm>

namespace ORB_SLAM
{

MapDatabase::MapDatabase(ORBVocabulary* vocab) {
    // Init varibles
    this->vocab = vocab;
    this->currentMapID = 0;
    this->maps = std::vector<Map*>();
}

Map* MapDatabase::getNewMap() {
    boost::mutex::scoped_lock lock(vocMutex);
    // Create map and new db
    Map* temp = new Map;
    KeyFrameDatabase* db = new KeyFrameDatabase(*vocab);
    // Set the db, and return the new object
    temp->SetKeyFrameDB(db);
    return temp;
}

void MapDatabase::addMap(Map* map) {
    boost::mutex::scoped_lock lock(mapMutex);
    // Reset flag on current map
    if(currentMapID > 0 && currentMapID < maps.size()+1)
        maps.at(currentMapID-1)->ResetUpdated();
    // Add map
    maps.push_back(map);
    currentMapID = maps.size();
}

Map* MapDatabase::getMap(int loc) {
    boost::mutex::scoped_lock lock(mapMutex);
    // Check that we are in range
    if(loc < 0 || loc >= (int)maps.size())
            return NULL;
    // Success
    return maps.at(loc);
}

bool MapDatabase::eraseMap(Map* m){
    boost::mutex::scoped_lock lock(mapMutex);
    // Check to see if it is the current one
    if(currentMapID > 0 && currentMapID < maps.size()+1 && m == maps.at(currentMapID-1))
        currentMapID = 0;
    // Delete it
    for (std::size_t i = 0; i != maps.size(); ++i) {
        // If a match is found delete it, and remove it from the  vector
        if(maps[i] == m) {
            Map* temp = maps[i];
            maps.erase(maps.begin()+i);
            delete temp;
            return true;
        }
    }
    return false;
}

void MapDatabase::removeMap(Map* m){
    boost::mutex::scoped_lock lock(mapMutex);
    // Check to see if it is the current one
    if(currentMapID > 0 && currentMapID < maps.size()+1 && m == maps.at(currentMapID-1))
        currentMapID = 0;
    // Delete it
    for (std::size_t i = 0; i != maps.size(); ++i) {
        // If a match is found delete it, and remove it from the  vector
        if(maps[i] == m) {
            maps.erase(maps.begin()+i);
            return;
        }
    }
}

bool MapDatabase::isContained(Map* m) {
    boost::mutex::scoped_lock lock(mapMutex);
    // Search for the map
    if(std::find(maps.begin(), maps.end(), m) != maps.end())
        return true;
    return false;
}

bool MapDatabase::setMap(Map* m){
    boost::mutex::scoped_lock lock(mapMutex);
    unsigned int id_new = 0;
    for (std::size_t i = 0; i != maps.size(); ++i) {
        if(maps[i]->getErased())
            continue;
        if(maps[i] == m) {
            id_new = i+1;
            break;
        }
    }
    if(id_new <= 0 || id_new > maps.size())
        return false;
    else {
        currentMapID = id_new;
        return true;
    }
}

Map* MapDatabase::getCurrent() {
    boost::mutex::scoped_lock lock(mapMutex);
    if(currentMapID > 0 && currentMapID < maps.size()+1)
        return maps.at(currentMapID-1);
    else
        return NULL;
}

int MapDatabase::getCurrentID() {
    boost::mutex::scoped_lock lock(mapMutex);
    return currentMapID;
}

std::vector<Map*> MapDatabase::getAll() {
    boost::mutex::scoped_lock lock(mapMutex);
    return maps;
}

ORBVocabulary* MapDatabase::getVocab() {
    boost::mutex::scoped_lock lock(vocMutex);
    return vocab;
}

Map* MapDatabase::getOldest(Map* m1, Map* m2) {
    boost::mutex::scoped_lock lock(mapMutex);
    for (std::size_t i = 0; i != maps.size(); ++i) {
        if(maps[i] == m1)
            return m1;
        if(maps[i] == m2)
            return m2;
    }
    return NULL;
}

} //namespace ORB_SLAM