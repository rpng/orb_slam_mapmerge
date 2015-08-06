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

#include "threads/MapMerging.h"

#include "util/Sim3Solver.h"
#include "util/Converter.h"
#include "util/Optimizer.h"
#include "util/ORBmatcher.h"

#include <ros/ros.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

MapMerging::MapMerging(MapDatabase *pMap): OrbThread(pMap) {}

void MapMerging::Run()
{
    ros::Rate r(400);
    while(ros::ok())
    {
        // Reset if needed
        ResetIfRequested();

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates
            if(DetectLoop())
            {
                // Compute similarity transformation [sR|t]
               if(ComputeSim3())
               {
                   ROS_INFO("ORB-SLAM - Map Merge Detected");
                   CorrectLoop();
                   ROS_INFO("ORB-SLAM - Done Merging Maps");
               }
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
        }
        // Sleep
        r.sleep();
    }
}

void MapMerging::InsertKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool MapMerging::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool MapMerging::DetectLoop()
{
    { 
        boost::mutex::scoped_lock lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpCurrentKF->SetErase();
        return false;
    }
   
   // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    DBoW2::BowVector CurrentBowVec = mpCurrentKF->GetBowVector();
    float minScore = 1;     
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        DBoW2::BowVector BowVec = pKF->GetBowVector();

        float score = mapDB->getVocab()->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }
    
    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs;
    
    // Loop through each map, and add candidates to the list
    for(size_t i=0; i<mapDB->getAll().size(); i++) {
        // Ignore keyframes from the current db, let the loop closer take care of that
        if(mapDB->getAll().at(i) != mapDB->getCurrent()) {
            vector<KeyFrame*> temp = mapDB->getAll().at(i)->GetKeyFrameDatabase()->DetectLoopCandidates(mpCurrentKF, minScore);
            vpCandidateKFs.insert(vpCandidateKFs.end(), temp.begin(), temp.end());
        }
    }

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframe to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    if(!mvpEnoughConsistentCandidates.empty())
        return true;

    // Else we have not found one
    mpCurrentKF->SetErase();
    return false;
}

bool MapMerging::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i]);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is successful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reaches max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();

                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);


                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10);

                // If optimization is successful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mpMatchedgScm = gScm;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void MapMerging::CorrectLoop()
{
    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLoopCloser->RequestStop();
    mpRelocalizer->RequestStop();
    
    // Get the newest and oldest map ordering
    Map* newest;
    Map* oldest = mapDB->getOldest(mpMatchedKF->getMap(), mpCurrentKF->getMap());
    
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    cv::Mat Tw2a;
    g2o::Sim3 g2oSab;
    cv::Mat Tbw1;
    
    // Check to see if we have a valid map returned
    if(oldest == NULL)
    {
        ROS_ERROR("ORB-SLAM - Invalid maps trying to merge");
        return;
    }
    // Figure out what the newest map is based on the returned old one
    else if(oldest == mpMatchedKF->getMap())
    {
        // Update map varaibles
        newest = mpCurrentKF->getMap();
        // Different poses we have to convert to and from
        Tw2a = mpCurrentKF->GetPoseInverse();
        g2oSab = mpMatchedgScm;
        Tbw1 = mpMatchedKF->GetPose();
    }
    // Flip the measurments if we are going oldest->new
    else
    {
        // Update map varaibles
        newest = mpMatchedKF->getMap();
        // Different poses we have to convert to and from
        Tw2a = mpMatchedKF->GetPoseInverse();
        g2oSab = mpMatchedgScm.inverse();
        Tbw1 = mpCurrentKF->GetPose();
    }
    
    // Create sim3 of  the world 2 global to its connecting keyframe
    cv::Mat Rw2a = Tw2a.rowRange(0,3).colRange(0,3);
    cv::Mat tw2a = Tw2a.rowRange(0,3).col(3);
    g2o::Sim3 g2oSw2a(Converter::toMatrix3d(Rw2a),Converter::toVector3d(tw2a),1.0);
    // Create sim3 of the world 1 relative frame to its global
    cv::Mat Rbw1 = Tbw1.rowRange(0,3).colRange(0,3);
    cv::Mat tbw1 = Tbw1.rowRange(0,3).col(3);
    g2o::Sim3 g2oSbw1(Converter::toMatrix3d(Rbw1),Converter::toVector3d(tbw1),1.0);

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();
    mpMatchedKF->UpdateConnections();
    
    // Print out the two poses
    // std::cout << "C1 = "<< std::endl << " "  << mpCurrentKF->GetPose() << std::endl << std::endl;
    // std::cout << "C2 = "<< std::endl << " "  << mpMatchedKF->GetPose() << std::endl << std::endl;
    
    // Loop through all keyframes
    size_t num_keys = newest->GetAllKeyFrames().size();
    for(size_t i=0; i<num_keys; i++)
    {
        // Get the keyframe
        KeyFrame* pKFi = newest->GetAllKeyFrames().at(i);
        cv::Mat Tiw2 = pKFi->GetPose();
        // Ensure we have a valid key frame
        if(!pKFi)
            continue;
        if(pKFi->isBad())
            continue;
        // Get the sim3 of pose of the current keyframe
        cv::Mat Riw2 = Tiw2.rowRange(0,3).colRange(0,3);
        cv::Mat tiw2 = Tiw2.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw2(Converter::toMatrix3d(Riw2),Converter::toVector3d(tiw2),1.0);
        // Converting from the keyframe's referance to the other global
        g2o::Sim3 g2oSiw1 = g2oSiw2* g2oSw2a*g2oSab*g2oSbw1;
        // Update sim3 solvers with the new solution
        CorrectedSim3[pKFi]=g2oSiw1;
        NonCorrectedSim3[pKFi]=g2oSiw2;
    }
    
    // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
    for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        
        // Update refs
        pKFi->setMap(oldest);
        oldest->AddKeyFrame(pKFi);
        oldest->GetKeyFrameDatabase()->add(pKFi);

        vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            MapPoint* pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                continue;
            // Update refs
            pMPi->setMap(oldest);
            oldest->AddMapPoint(pMPi);
            // Project with non-corrected pose and project back with corrected pose
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            // Going from w2 to w1 and then maping the 3x1 vector ontop of it 
            // https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/sim3/sim3.h#L136-L138
            g2o::Sim3 g2oSw1w2 = g2oSbw1.inverse()*g2oSab.inverse()* g2oSw2a.inverse();
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oSw1w2.map(eigP3Dw);
            // Update the mappoint with the corrected cords
            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw);
            pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
            pMPi->mnCorrectedReference = pKFi->mnId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();
        
        // Debug
        //std::cout << "Trans is: " << eigt << std::endl;
        //std::cout << "Rots is: " << eigR << std::endl;
        //std::cout << "Scale is: " << s << std::endl;

        eigt *=(1./s); //[R t/s;0 1]

        // Update key frame position
        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);        
        pKFi->SetPose(correctedTiw);

        // Make sure connections are updated
        pKFi->UpdateConnections();
    } 
    
    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
        {
            MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
            MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
            if(pCurMP)
                pCurMP->Replace(pLoopMP);
            else
            {
                mpCurrentKF->AddMapPoint(pLoopMP,i);
                pLoopMP->AddObservation(mpCurrentKF,i);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }
    
    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);

    //Add edge
    mpCurrentKF->AddLoopEdge(mpMatchedKF);
    mpMatchedKF->AddLoopEdge(mpCurrentKF);

     // Update the current map
    mapDB->setMap(oldest);
    // Remove the map from our list, but keep the data
//    mapDB->removeMap(newest);
    newest->setErased(true);

    // Force the tracker to relocalize the camera in the updated map
    mpTracker->ForceInlineRelocalisation();

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();
    mpLoopCloser->Release();

    // Update the local last loop id var
    mLastLoopKFid = mpCurrentKF->mnId;
}

void MapMerging::SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4);
    }
}

void MapMerging::Release()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    boost::mutex::scoped_lock lock2(mMutexLoopQueue);
    mbStopped = false;
    mbStopRequested = false;
    // We do not need to delete keyframes any more because they are linked to maps
    //    for(list<KeyFrame*>::iterator lit = mlpLoopKeyFrameQueue.begin(), lend=mlpLoopKeyFrameQueue.end(); lit!=lend; lit++)
    //        delete *lit;
    mlpLoopKeyFrameQueue.clear();
}

void MapMerging::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    boost::mutex::scoped_lock lock2(mMutexLoopQueue);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}
} //namespace ORB_SLAM