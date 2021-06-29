/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Config.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef pair<set<boost::interprocess::offset_ptr<KeyFrame> >,int> ConsistentGroup;    
    typedef map<boost::interprocess::offset_ptr<KeyFrame> ,g2o::Sim3,std::less<boost::interprocess::offset_ptr<KeyFrame> >,
        Eigen::aligned_allocator<std::pair<boost::interprocess::offset_ptr<KeyFrame>  const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(boost::interprocess::offset_ptr<KeyFrame> pKF);

    void RequestReset();
    void RequestResetActiveMap(boost::interprocess::offset_ptr<Map>  pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(boost::interprocess::offset_ptr<Map>  pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef REGISTER_TIMES
    double timeDetectBoW;

    std::vector<double> vTimeBoW_ms;
    std::vector<double> vTimeSE3_ms;
    std::vector<double> vTimePRTotal_ms;

    std::vector<double> vTimeLoopFusion_ms;
    std::vector<double> vTimeLoopEssent_ms;
    std::vector<double> vTimeLoopTotal_ms;

    std::vector<double> vTimeMergeFusion_ms;
    std::vector<double> vTimeMergeBA_ms;
    std::vector<double> vTimeMergeTotal_ms;

    std::vector<double> vTimeFullGBA_ms;
    std::vector<double> vTimeMapUpdate_ms;
    std::vector<double> vTimeGBATotal_ms;
#endif

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<boost::interprocess::offset_ptr<KeyFrame> > &vpBowCand, boost::interprocess::offset_ptr<KeyFrame>  &pMatchedKF, boost::interprocess::offset_ptr<KeyFrame>  &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs);
    int FindMatchesByProjection(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<boost::interprocess::offset_ptr<MapPoint> > &spMatchedMPinOrigin, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints,
                                vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints);
    void SearchAndFuse(const vector<boost::interprocess::offset_ptr<KeyFrame> > &vConectedKFs, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    boost::interprocess::offset_ptr<Map>  mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<boost::interprocess::offset_ptr<KeyFrame> > mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    boost::interprocess::offset_ptr<KeyFrame>  mpCurrentKF;
    boost::interprocess::offset_ptr<KeyFrame>  mpLastCurrentKF;
    boost::interprocess::offset_ptr<KeyFrame>  mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > mvpEnoughConsistentCandidates;
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > mvpCurrentConnectedKFs;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpCurrentMatchedPoints;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    boost::interprocess::offset_ptr<Map>  mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    boost::interprocess::offset_ptr<KeyFrame>  mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    boost::interprocess::offset_ptr<KeyFrame>  mpLoopMatchedKF;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpLoopMPs;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    boost::interprocess::offset_ptr<KeyFrame>  mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    boost::interprocess::offset_ptr<KeyFrame>  mpMergeMatchedKF;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpMergeMPs;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpMergeMatchedMPs;
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;



    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
