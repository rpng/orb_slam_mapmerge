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

#include "threads/Relocalization.h"
#include "threads/Tracking.h"

#include "types/Frame.h"
#include "types/KeyFrame.h"
#include "types/MapDatabase.h"

#include "util/ORBmatcher.h"
#include "util/Converter.h"
#include "util/Initializer.h"
#include "util/Optimizer.h"
#include "util/PnPsolver.h"

#include <ros/ros.h>

namespace ORB_SLAM
{

Relocalization::Relocalization(MapDatabase *pMap):
    OrbThread(pMap),  acceptingFrames(true), mCurrentFrame(NULL), isSuccessfull(false), mapMatch(NULL)
{
}
    
void Relocalization::Run()
{
    ros::Rate r(200);
    while(ros::ok())
    {
        // Reset if needed
        ResetIfRequested();

        bool should_proccess = false;
        {
            boost::mutex::scoped_lock lock(mMutexFrame);
            should_proccess = (mCurrentFrame != NULL);
        }
        // Check if we have a new frame
        if(should_proccess)
        {
            // Check if we have already been successfull
            // If so then we do not want to overwrite those results
            if(!isSuccess())
            {
                Relocalisation();
            }
        }
        
        // Safe area to stop
        if(stopRequested())
        {
            Stop();
            ros::Rate r2(200);
            while(isStopped() && ros::ok())
            {
                r2.sleep();
            }
            setAcceptingFrames(true);
        }
        // Sleep
        r.sleep();
    }
}

void Relocalization::AddFrame(Frame* newFrame)
{
    // Check if we are accepting keyframes, or have had a success
    if(!isAcceptingFrames() || isSuccess()) {
        delete newFrame;
        return;
    }
    boost::mutex::scoped_lock lock(mMutexFrame);
    // We are going to add a frame, so do not accept
    setAcceptingFrames(false);
    // Delete the old frame if not null
    if(mCurrentFrame != NULL)
        delete mCurrentFrame;
    // Set the current frame to try to relocalize at
    mCurrentFrame = newFrame;
}


void Relocalization::Relocalisation()
{
    // We are not accepting frames
    // This allows the current frame to be thread safe
    setAcceptingFrames(false);

    // Compute Bag of Words Vector
    mCurrentFrame->ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;

    int count =0;
    // Add all keyframe candidates we have
    // More recent maps are more likely, so we loop backwards
    for(int i=mapDB->getAll().size()-1; i>=0; i--) {
        // Ensure we are erased
        if(mapDB->getAll().at(i)->getErased())
            continue;
        // Get all keyframes that match the current one
        vector<KeyFrame*> temp = mapDB->getAll().at(i)->GetKeyFrameDatabase()->DetectRelocalisationCandidates(mCurrentFrame);
        vpCandidateKFs.insert(vpCandidateKFs.end(), temp.begin(), temp.end());
        // Increment out count
        count++;
    }

    // If we don't have any maps, we don't need to relocalize
    if(count == 0) {
        mpTracker->ResetRelocalisationRequested();
        RequestStop();
        setAcceptingFrames(true);
        return;
    }

    // Do not continue if we have no candidates
    if(vpCandidateKFs.empty())
    {
        setAcceptingFrames(true);
        return;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);
    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;
    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,*mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(*mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);
    int match = -1;
    
    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame->mTcw);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame->mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(mCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame->mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame->mvbOutlier[io])
                        mCurrentFrame->mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(*mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame->mvpMapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(*mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(mCurrentFrame);

                                for(size_t io =0; io<mCurrentFrame->mvbOutlier.size(); io++)
                                    if(mCurrentFrame->mvbOutlier[io])
                                        mCurrentFrame->mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    match = i;
                    break;
                }
            }
        }
    }

    // If we do not have a match accept keyframes
    if(!bMatch)
    {
        setAcceptingFrames(true);
        return;
    }
    // Success, store the keyframe and map
    else
    {         
        // If we have a match id, get its map, and update the mapDB's current map
        if(match != -1 && vpCandidateKFs[match]->getMap() != NULL)
        {
            ROS_INFO("ORB-SLAM - Relocalization Match Found");
            {
                boost::mutex::scoped_lock lock(mMutexSuccessCheck);
                mapMatch = vpCandidateKFs[match]->getMap();
                isSuccessfull = true;
            }
        }
        else
        {
            RequestReset();
            ROS_WARN("ORB-SLAM - Unable to find the map linked to relocalized keyframe.");
        }
    }

}


bool Relocalization::isAcceptingFrames()
{
    boost::mutex::scoped_lock lock(mMutexAcceptFrames);
    return acceptingFrames;
}
        
void Relocalization::setAcceptingFrames(bool val)
{
    boost::mutex::scoped_lock lock(mMutexAcceptFrames);
    acceptingFrames = val;
}

bool Relocalization::isSuccess()
{
    boost::mutex::scoped_lock lock(mMutexSuccessCheck);
    return isSuccessfull;
}

bool Relocalization::relocalizeIfSuccessfull()
{
        // Check if we have been succesfull
        if(!isSuccess())
            return false;
        boost::mutex::scoped_lock lock(mMutexFrame);
        boost::mutex::scoped_lock lock2(mMutexSuccessCheck);
        // Set the map
        // Set map handles making sure the map we want to set is not erased
        if(mapDB->setMap(mapMatch))
        {
            // We do not need to update the map
            mapMatch->ResetUpdated();
            // We are relocalized, reset it
            mpTracker->ResetRelocalisationRequested();
            mpTracker->SetRelocalisationFrame(mCurrentFrame);
            // We should stop, and reset after a success
            RequestReset();
            RequestStop();
            // Success
            ROS_INFO("ORB-SLAM - Successful relocalisation to old map. (thread)");
            return true;
        }
        // We are not successfull, reset everything
        RequestReset();
        // We have an invalid map
        return false;
}

void Relocalization::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexSuccessCheck);
    boost::mutex::scoped_lock lock2(mMutexFrame);
    boost::mutex::scoped_lock lock3(mMutexReset);
    if(mbResetRequested)
    {
        // If there is a frame delete it
         if(mCurrentFrame != NULL)
                delete mCurrentFrame;
        // Reset vars
        mCurrentFrame = NULL;
        isSuccessfull = false;
        mapMatch = NULL;
        // Accept frames
        setAcceptingFrames(true);
        // Reset reset var
        mbResetRequested=false;
    }
}

}

