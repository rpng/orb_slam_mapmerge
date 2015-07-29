/**
* This file is part of ORB-SLAM.
* Orginal source code taken and adapted from:
* http://noobtuts.com/cpp/frames-per-second
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

#include "util/FpsCounter.h"

#include <boost/thread.hpp>

// Constructor
FpsCounter::FpsCounter() :initial_time(time(NULL)), m_fpscount(0), m_fps(0)
{
}

// Update
void FpsCounter::update()
{
    // increase the counter by one
    m_fpscount++;

    // one second elapsed? (= 1000 milliseconds)
    if (difftime(time(NULL), initial_time) >= 1)
    {
        // save the current counter value to m_fps
        {
            boost::mutex::scoped_lock lock(mMutexFps);
            m_fps = m_fpscount;
        }
        // reset the counter and the interval
        m_fpscount = 0;
        initial_time = time(NULL);
    }
}

// Get fps
unsigned int FpsCounter::get()
{
    boost::mutex::scoped_lock lock(mMutexFps);
    return m_fps;
}