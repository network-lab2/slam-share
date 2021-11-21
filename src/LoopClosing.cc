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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM3
{

LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
    mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<boost::interprocess::offset_ptr<KeyFrame> >(NULL);
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        //NEW LOOP AND MERGE DETECTION ALGORITHM
        //----------------------------
        if(CheckNewKeyFrames())
        {
            std::cout<<"New KeyFrames\n";
            if(mpLastCurrentKF)
            {
                //OLD CODE:
                //mpLastCurrentKF->mvpLoopCandKFs.clear();
                //mpLastCurrentKF->mvpMergeCandKFs.clear();
                //NEW CODE:
                mpLastCurrentKF->mvpLoopCandKFs->clear();
                mpLastCurrentKF->mvpMergeCandKFs->clear();
                std::cout<<"===== Loop closing, if mpLastCurrentKF\n";
            }
#ifdef REGISTER_TIMES
            timeDetectBoW = 0;
            std::chrono::steady_clock::time_point time_StartDetectBoW = std::chrono::steady_clock::now();
#endif
            bool bDetected = NewDetectCommonRegions();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndDetectBoW = std::chrono::steady_clock::now();
            double timeDetect = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndDetectBoW - time_StartDetectBoW).count();
            double timeDetectSE3 = timeDetect - timeDetectBoW;

            if(timeDetectBoW > 0)
            {
                vTimeBoW_ms.push_back(timeDetectBoW);
            }
            vTimeSE3_ms.push_back(timeDetectSE3);
            vTimePRTotal_ms.push_back(timeDetect);
#endif

            if(bDetected)
            {
                std::cout<<"****------***** NewDetectCommonRegions returned true.____---"<<std::endl;
                if(mbMergeDetected)
                {
                    if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
                    {
                        cout << "IMU is not initilized, merge is aborted" << endl;
                    }
                    else
                    {
                        Verbose::PrintMess("*Merged detected", Verbose::VERBOSITY_QUIET);
                        std::cout<<"Merged detected -----\n";
                        Verbose::PrintMess("Number of KFs in the current map: " + to_string(mpCurrentKF->GetMap()->KeyFramesInMap()), Verbose::VERBOSITY_DEBUG);
                        cv::Mat mTmw = mpMergeMatchedKF->GetPose();
                        g2o::Sim3 gSmw2(Converter::toMatrix3d(mTmw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTmw.rowRange(0, 3).col(3)),1.0);
                        cv::Mat mTcw = mpCurrentKF->GetPose();
                        g2o::Sim3 gScw1(Converter::toMatrix3d(mTcw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcw.rowRange(0, 3).col(3)),1.0);
                        g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                        g2o::Sim3 gSw1m = mg2oMergeSlw;

                        mSold_new = (gSw2c * gScw1);

                        if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                   mpCurrentKF->GetMap()->GetIniertialBA1()) // TODO, maybe with GetIniertialBA1
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0)=0;
                                phi(1)=0;
                                mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                            }
                        }

                        mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                        mg2oMergeScw = mg2oMergeSlw;

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif
                        if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO)
                            MergeLocal2();
                        else
                            MergeLocal();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();
                        double timeMerge = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMerge - time_StartMerge).count();
                        vTimeMergeTotal_ms.push_back(timeMerge);
#endif
                    }

                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    // Reset all variables
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected = false;

                    if(mbLoopDetected)
                    {
                        // Reset Loop variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }

                }

                if(mbLoopDetected)
                {
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(0);


                    Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

                    mg2oLoopScw = mg2oLoopSlw;
                    if(mpCurrentKF->GetMap()->IsInertial())
                    {
                        cv::Mat Twc = mpCurrentKF->GetPoseInverse();
                        g2o::Sim3 g2oTwc(Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(Twc.rowRange(0, 3).col(3)),1.0);
                        g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());

                        if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
                        {
                            if(mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2())
                                {
                                    phi(0)=0;
                                    phi(1)=0;
                                    g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                    mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                }
                            }

                            mvpLoopMapPoints = mvpLoopMPs;//*mvvpLoopMapPoints[nCurrentIndex];

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                            CorrectLoop();
#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                            double timeLoop = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLoop - time_StartLoop).count();
                            vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                        }
                        else
                        {
                            cout << "BAD LOOP!!!" << endl;
                        }
                    }
                    else
                    {
                        mvpLoopMapPoints = mvpLoopMPs;
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                        CorrectLoop();

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                        double timeLoop = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLoop - time_StartLoop).count();
                        vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                    }

                    // Reset all variables
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected = false;
                }

            }
            mpLastCurrentKF = mpCurrentKF;
        }

        ResetIfRequested();

        if(CheckFinish()){
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    std::cout<<"mMutexLoop queue passed\n";
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);

    std::cout<<"Added to mlpLoopKeyFrameQueue\n";
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions()
{
    std::cout<<"New detect common regions 1.\n";
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
        mpCurrentKF->mbCurrentPlaceRecognition = true;

        mpLastMap = mpCurrentKF->GetMap();
    }

    std::cout<<"New detect common regions 2\n";

    if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA1())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if(mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    std::cout<<"New detect common regions 3.\n";

    //Check the last candidates with geometric validation
    // Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial = false;

    if(mnLoopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse();
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScw = gScl * mg2oLoopSlw;
        int numProjMatches = 0;
        vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);
        if(bCommonRegion)
        {

            bLoopDetectedInKF = true;

            mnLoopNumCoincidences++;
            mpLoopLastCurrentKF->SetErase();
            mpLoopLastCurrentKF = mpCurrentKF;
            mg2oLoopSlw = gScw;
            mvpLoopMatchedMPs = vpMatchedMPs;


            mbLoopDetected = mnLoopNumCoincidences >= 3;
            mnLoopNumNotFound = 0;

            if(!mbLoopDetected)
            {
                cout << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;

            mnLoopNumNotFound++;
            if(mnLoopNumNotFound >= 2)
            {

                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMPs.clear();
                mnLoopNumNotFound = 0;
            }

        }
    }

    std::cout<<"New detect common regions 4.\n";

    //Merge candidates
    bool bMergeDetectedInKF = false;
    if(mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse();
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
        if(bCommonRegion)
        {
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++;
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3;
        }
        else
        {
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if(mnMergeNumNotFound >= 2)
            {

                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
            }


        }
    }

    std::cout<<"New detect common regions 5.\n";

    if(mbMergeDetected || mbLoopDetected)
    {
        std::cout<<"New detect common regions 6.\n";
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    const vector<boost::interprocess::offset_ptr<KeyFrame> > vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;

    std::cout<<"New detect common regions 7.\n";

    // Extract candidates from the bag of words
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpMergeBowCand, vpLoopBowCand;
    if(!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartDetectBoW = std::chrono::steady_clock::now();
#endif
        // Search in BoW
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndDetectBoW = std::chrono::steady_clock::now();
        timeDetectBoW = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndDetectBoW - time_StartDetectBoW).count();
#endif
    }

    std::cout<<"New detect common regions 8.\n";

    if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
    }
    // Merge candidates

    if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
    }
    std::cout<<"New detect common regions 9.\n";
    mpKeyFrameDB->add(mpCurrentKF);
    std::cout<<"New detect common regions 10.\n";
    if(mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs)
{
    set<boost::interprocess::offset_ptr<MapPoint> > spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);


    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if(nNumProjMatches >= nProjMatches)
    {
        cv::Mat mScw = Converter::toCvMat(gScw);
        cv::Mat mTwm = pMatchedKF->GetPoseInverse();
        g2o::Sim3 gSwm(Converter::toMatrix3d(mTwm.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTwm.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial
        if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale=false;
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);



        if(numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(Converter::toMatrix3d(mScw.rowRange(0, 3).colRange(0, 3)),
                           Converter::toVector3d(mScw.rowRange(0, 3).col(3)),1.0);

            vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
            if(nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<boost::interprocess::offset_ptr<KeyFrame> > &vpBowCand, boost::interprocess::offset_ptr<KeyFrame>  &pMatchedKF2, boost::interprocess::offset_ptr<KeyFrame>  &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                             int &nNumCoincidences, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs)
{
    std::cout<<"DetectCommonRegionsFromBoW 1\n";
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<boost::interprocess::offset_ptr<KeyFrame> > spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    std::cout<<"DetectCommonRegionsFromBoW 2\n";

    int nNumCovisibles = 5;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);
    int nNumGuidedMatching = 0;

    boost::interprocess::offset_ptr<KeyFrame>  pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > vpBestMapPoints;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > vpBestMatchedMapPoints;

    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    std::cout<<"DetectCommonRegionsFromBoW 3\n";

    int index = 0;
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpBowCand)
    {
        if(!pKFi || pKFi->isBad())
            continue;


        // Current KF against KF with covisibles version
        std::vector<boost::interprocess::offset_ptr<KeyFrame> > vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(vpCovKFi[0]);
        vpCovKFi[0] = pKFi;

        std::vector<std::vector<boost::interprocess::offset_ptr<MapPoint> > > vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<boost::interprocess::offset_ptr<MapPoint> > spMatchedMPi;
        int numBoWMatches = 0;

        boost::interprocess::offset_ptr<KeyFrame>  pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedPoints = std::vector<boost::interprocess::offset_ptr<MapPoint> >(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
        std::vector<boost::interprocess::offset_ptr<KeyFrame> > vpKeyFrameMatchedMP = std::vector<boost::interprocess::offset_ptr<KeyFrame> >(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<KeyFrame> >(NULL));

        int nIndexMostBoWMatchesKF=0;
        std::cout<<"DetectCommonRegionsFromBoW 4\n";
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;
            std::cout<<"DetectCommonRegionsFromBoW 4.1\n";
            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            std::cout<<"DetectCommonRegionsFromBoW 4.2\n";
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
            std::cout<<"DetectCommonRegionsFromBoW 4.3\n";
        }
        std::cout<<"DetectCommonRegionsFromBoW 5\n";
        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }

            for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            {
                boost::interprocess::offset_ptr<MapPoint>  pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                    continue;

                if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]= pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }
        std::cout<<"DetectCommonRegionsFromBoW 6\n";

        if(!bAbortByNearKF && numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation

            bool bFixedScale = mbFixScale;
            if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;

            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            std::cout<<"DetectCommonRegionsFromBoW 6.1\n";

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            cv::Mat mTcm;
            while(!bConverge && !bNoMore)
            {
                mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
            }
            std::cout<<"DetectCommonRegionsFromBoW 6.2\nmTcm size"<<mTcm.size()<<" bconverge "<<bConverge<<std::endl;
            if(bConverge)
            {
                vpCovKFi.clear();
                std::cout<<"DetectCommonRegionsFromBoW 6.3\n";
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                int nInitialCov = vpCovKFi.size();
                vpCovKFi.push_back(pMostBoWMatchesKF);
                std::cout<<"DetectCommonRegionsFromBoW 6.4\n";

                set<boost::interprocess::offset_ptr<KeyFrame> > spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                set<boost::interprocess::offset_ptr<MapPoint> > spMapPoints;
                vector<boost::interprocess::offset_ptr<MapPoint> > vpMapPoints;
                vector<boost::interprocess::offset_ptr<KeyFrame> > vpKeyFrames;
                for(boost::interprocess::offset_ptr<KeyFrame>  pCovKFi : vpCovKFi)
                {
                    for(boost::interprocess::offset_ptr<MapPoint>  pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                            continue;

                        if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }
                std::cout<<"DetectCommonRegionsFromBoW 6.5\n";
                g2o::Sim3 gScm(Converter::toMatrix3d(solver.GetEstimatedRotation()),Converter::toVector3d(solver.GetEstimatedTranslation()),solver.GetEstimatedScale());
                g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                cv::Mat mScw = Converter::toCvMat(gScw);
                std::cout<<"DetectCommonRegionsFromBoW 7\n";

                vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
                vector<boost::interprocess::offset_ptr<KeyFrame> > vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<KeyFrame> >(NULL));
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                std::cout<<"DetectCommonRegionsFromBoW 8\n";
                if(numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale=false;

                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                    if(numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        cv::Mat mScw = Converter::toCvMat(gScw);

                        vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMP;
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
                        int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if(numProjOptMatches >= nProjOptMatches)
                        {
                            int nNumKFs = 0;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<boost::interprocess::offset_ptr<KeyFrame> > vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                            int j = 0;
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            {
                                boost::interprocess::offset_ptr<KeyFrame>  pKFj = vpCurrentCovKFs[j];
                                cv::Mat mTjc = pKFj->GetPose() * mpCurrentKF->GetPoseInverse();
                                g2o::Sim3 gSjc(Converter::toMatrix3d(mTjc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTjc.rowRange(0, 3).col(3)),1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<boost::interprocess::offset_ptr<MapPoint> > vpMatchedMPs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if(bValid)
                                {
                                    nNumKFs++;
                                }

                                j++;
                            }

                            if(nNumKFs < 3)
                            {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if(nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }


                        }

                    }

                }
            }
            std::cout<<"DetectCommonRegionsFromBoW 9\n";
            //delete solver;
        }
        index++;
        std::cout<<"DetectCommonRegionsFromBoW 9.1\n";
    }

    std::cout<<"DetectCommonRegionsFromBoW 10\n";

    if(nBestMatchesReproj > 0)
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw;
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for(int i=0; i<vnStage.size(); ++i)
        {
            if(vnStage[i] > maxStage)
            {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }

    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMPs)
{
    set<boost::interprocess::offset_ptr<MapPoint> > spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    if(nNumProjMatches >= nProjMatches)
    {

        return true;
    }

    return false;
}

int LoopClosing::FindMatchesByProjection(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, boost::interprocess::offset_ptr<KeyFrame>  pMatchedKFw, g2o::Sim3 &g2oScw,
                                         set<boost::interprocess::offset_ptr<MapPoint> > &spMatchedMPinOrigin, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints,
                                         vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatchedMapPoints)
{
    int nNumCovisibles = 5;
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<boost::interprocess::offset_ptr<KeyFrame> > spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<boost::interprocess::offset_ptr<KeyFrame> > spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    for(int i=0; i<nInitialCov; ++i)
    {
        vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
        int nInserted = 0;
        int j = 0;
        while(j < vpKFs.size() && nInserted < nNumCovisibles)
        {
            if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
            {
                spCheckKFs.insert(vpKFs[j]);
                ++nInserted;
            }
            ++j;
        }
        vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
    }
    set<boost::interprocess::offset_ptr<MapPoint> > spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpCovKFm)
    {
        for(boost::interprocess::offset_ptr<MapPoint>  pMPij : pKFi->GetMapPointMatches())
        {
            if(!pMPij || pMPij->isBad())
                continue;

            if(spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    cv::Mat mScw = Converter::toCvMat(g2oScw);

    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches;
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue


    // If a Global Bundle Adjustment is running, abort it
    cout << "Request GBA abort" << endl;
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            cout << "GBA running... Abort!" << endl;
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    cout << "start updating connections" << endl;
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oLoopScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    boost::interprocess::offset_ptr<Map>  pLoopMap = mpCurrentKF->GetMap();

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for(vector<boost::interprocess::offset_ptr<KeyFrame> >::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            boost::interprocess::offset_ptr<KeyFrame>  pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            boost::interprocess::offset_ptr<KeyFrame>  pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<boost::interprocess::offset_ptr<MapPoint> > vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                boost::interprocess::offset_ptr<MapPoint>  pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

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

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Correct velocity according to orientation correction
            if(bImuInit)
            {
                Eigen::Matrix3d Rcor = eigR.transpose()*g2oSiw.rotation().toRotationMatrix();
                pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity());
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        // TODO Check this index increasement
        pLoopMap->IncreaseChangeIndex();


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)
        {
            if(mvpLoopMatchedMPs[i])
            {
                boost::interprocess::offset_ptr<MapPoint>  pLoopMP = mvpLoopMatchedMPs[i];
                boost::interprocess::offset_ptr<MapPoint>  pCurMP = mpCurrentKF->GetMapPoint(i);
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
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<boost::interprocess::offset_ptr<KeyFrame> , set<boost::interprocess::offset_ptr<KeyFrame> > > LoopConnections;

    for(vector<boost::interprocess::offset_ptr<KeyFrame> >::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKFi = *vit;
        vector<boost::interprocess::offset_ptr<KeyFrame> > vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<boost::interprocess::offset_ptr<KeyFrame> >::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<boost::interprocess::offset_ptr<KeyFrame> >::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = mbFixScale;
    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
        bFixedScale=false;


    if(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
    }
    else
    {
        Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
    }

    mpAtlas->InformNewBigChange();

    // Add loop edge
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
    if(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
    {
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;

        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
}

void LoopClosing::MergeLocal()
{
    std::cout<<"Merge local 1\n";
    Verbose::PrintMess("MERGE: Merge Visual detected!!!!", Verbose::VERBOSITY_NORMAL);

    int numTemporalKFs = 15;

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    boost::interprocess::offset_ptr<KeyFrame>  pNewChild;
    boost::interprocess::offset_ptr<KeyFrame>  pNewParent;

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpLocalCurrentWindowKFs;
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpMergeConnectedKFs;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    Verbose::PrintMess("MERGE: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }
     std::cout<<"Merge local 2\n";

    Verbose::PrintMess("MERGE: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    Verbose::PrintMess("MERGE: Local Map stopped", Verbose::VERBOSITY_DEBUG);

    mpLocalMapper->EmptyQueue();

    // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
    // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
    boost::interprocess::offset_ptr<Map>  pCurrentMap = mpCurrentKF->GetMap();
    boost::interprocess::offset_ptr<Map>  pMergeMap = mpMergeMatchedKF->GetMap();

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
    set<boost::interprocess::offset_ptr<KeyFrame> > spLocalWindowKFs;
    //Get MPs in the welding area from the current map
    set<boost::interprocess::offset_ptr<MapPoint> > spLocalWindowMPs;
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKFi = mpCurrentKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<boost::interprocess::offset_ptr<MapPoint> > spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }

        pKFi = mpCurrentKF->mNextKF;
        while(pKFi)
        {
            spLocalWindowKFs.insert(pKFi);

            set<boost::interprocess::offset_ptr<MapPoint> > spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }
    }
    else
    {
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    const int nMaxTries = 3;
    int nNumTries = 0;
    while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<boost::interprocess::offset_ptr<KeyFrame> > vpNewCovKFs;
        vpNewCovKFs.empty();
        for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
        {
            vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(boost::interprocess::offset_ptr<KeyFrame>  pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        set<boost::interprocess::offset_ptr<MapPoint> > spMPs = pKFi->GetMapPoints();
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }
     std::cout<<"Merge local 3\n";

    set<boost::interprocess::offset_ptr<KeyFrame> > spMergeConnectedKFs;
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKFi = mpMergeMatchedKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while(pKFi)
        {
            spMergeConnectedKFs.insert(pKFi);
        }
    }
    else
    {
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }
    vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    nNumTries = 0;
    while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<boost::interprocess::offset_ptr<KeyFrame> > vpNewCovKFs;
        for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
        {
            vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(boost::interprocess::offset_ptr<KeyFrame>  pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    set<boost::interprocess::offset_ptr<MapPoint> > spMapPointMerge;
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
    {
        set<boost::interprocess::offset_ptr<MapPoint> > vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
    }

    vector<boost::interprocess::offset_ptr<MapPoint> > vpCheckFuseMapPoint;
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    std::cout<<"Merge local 4\n";

    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat twc = Twc.rowRange(0,3).col(3);
    g2o::Sim3 g2oNonCorrectedSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    g2o::Sim3 g2oCorrectedScw = mg2oMergeScw;

    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
    vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;

    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        g2o::Sim3 g2oCorrectedSiw;

        if(pKFi!=mpCurrentKF)
        {
            cv::Mat Tiw = pKFi->GetPose();
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi]=g2oSiw;

            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2oCorrectedSiw = g2oSic*mg2oMergeScw;
            vCorrectedSim3[pKFi]=g2oCorrectedSiw;
        }
        else
        {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        //pKFi->mTcwMerge  = pKFi->GetPose();
        pKFi->GetPose().copyTo(pKFi->mTcwMerge);
        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        pKFi->mfScale = s;
        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        //pKFi->mTcwMerge = correctedTiw;
        correctedTiw.copyTo(pKFi->mTcwMerge);

        if(pCurrentMap->isImuInitialized())
        {
            Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
            cv::Mat temp1 = Converter::toCvMat(Rcor)*pKFi->GetVelocity();
            temp1.copyTo(pKFi->mVwbMerge);
            //pKFi->mVwbMerge = Converter::toCvMat(Rcor)*pKFi->GetVelocity();
        }

    }

     std::cout<<"Merge local 5\n";
    for(boost::interprocess::offset_ptr<MapPoint>  pMPi : spLocalWindowMPs)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        boost::interprocess::offset_ptr<KeyFrame>  pKFref = pMPi->GetReferenceKeyFrame();
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        cv::Mat P3Dw = pMPi->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));
        Eigen::Matrix3d eigR = g2oCorrectedSwi.rotation().toRotationMatrix();
        Eigen::Matrix3d Rcor = eigR * g2oNonCorrectedSiw.rotation().toRotationMatrix();

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);

        //std::cout<<"Size of cvCorrectedP3Dw: (mPosMerge) "<<cvCorrectedP3Dw.size()<<" element size: "<<cvCorrectedP3Dw.elemSize()<<std::endl;

        //pMPi->mPosMerge = cvCorrectedP3Dw;
        cvCorrectedP3Dw.copyTo(pMPi->mPosMerge);
        

        //pMPi->mNormalVectorMerge = Converter::toCvMat(Rcor) * pMPi->GetNormal();
        cv::Mat temp_mat = Converter::toCvMat(Rcor) * pMPi->GetNormal();
        //std::cout<<"Size of temp_map: (mNormalVectorMerge) "<<temp_mat.size()<<" element size: "<<temp_mat.elemSize()<<std::endl;
        temp_mat.copyTo(pMPi->mNormalVectorMerge);
    }

     std::cout<<"Merge local 6\n";
    {
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

        for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
            {
                continue;
            }

            //pKFi->mTcwBefMerge = pKFi->GetPose();
            //pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->GetPose().copyTo(pKFi->mTcwBefMerge);
            pKFi->GetPoseInverse().copyTo(pKFi->mTwcBefMerge);
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            pKFi->UpdateMap(pMergeMap);
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            pMergeMap->AddKeyFrame(pKFi);
            pCurrentMap->EraseKeyFrame(pKFi);

            if(pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
            }
        }

        for(boost::interprocess::offset_ptr<MapPoint>  pMPi : spLocalWindowMPs)
        {
            if(!pMPi || pMPi->isBad())
                continue;

            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            pMPi->UpdateMap(pMergeMap);
            pMergeMap->AddMapPoint(pMPi);
            pCurrentMap->EraseMapPoint(pMPi);
        }

        mpAtlas->ChangeMap(pMergeMap);
        mpAtlas->SetMapBad(pCurrentMap);
        pMergeMap->IncreaseChangeIndex();
    }

     std::cout<<"Merge local 7\n";

    //Rebuild the essential graph in the local window
    pCurrentMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpCurrentKF->ChangeParent(mpMergeMatchedKF);
    while(pNewChild )
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        boost::interprocess::offset_ptr<KeyFrame>  pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);

        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }

    //Update the connections between the local window
    mpMergeMatchedKF->UpdateConnections();

    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    // Update connectivity
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

     std::cout<<"Merge local 8\n";

    bool bStop = false;
    Verbose::PrintMess("MERGE: Start local BA ", Verbose::VERBOSITY_DEBUG);
    vpLocalCurrentWindowKFs.clear();
    vpMergeConnectedKFs.clear();
    std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
    std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
    if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)
    {
        Optimizer::MergeInertialBA(mpLocalMapper->GetCurrKF(),mpMergeMatchedKF,&bStop, mpCurrentKF->GetMap(),vCorrectedSim3);
    }
    else
    {
        Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    Verbose::PrintMess("MERGE: Finish the LBA", Verbose::VERBOSITY_DEBUG);

     std::cout<<"Merge local 9\n";

    ////
    //Update the non critical area from the current map to the merged map
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    vector<boost::interprocess::offset_ptr<MapPoint> > vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

    if(vpCurrentMapKFs.size() == 0)
    {
        Verbose::PrintMess("MERGE: There are not KFs outside of the welding area", Verbose::VERBOSITY_DEBUG);
    }
    else
    {
        Verbose::PrintMess("MERGE: Calculate the new position of the elements outside of the window", Verbose::VERBOSITY_DEBUG);
        //Apply the transformation
        {
            if(mpTracker->mSensor == System::MONOCULAR)
            {
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

                for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpCurrentMapKFs)
                {
                    if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }

                    g2o::Sim3 g2oCorrectedSiw;

                    cv::Mat Tiw = pKFi->GetPose();
                    cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
                    cv::Mat tiw = Tiw.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
                    //Pose without correction
                    vNonCorrectedSim3[pKFi]=g2oSiw;

                    cv::Mat Tic = Tiw*Twc;
                    cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                    cv::Mat tic = Tic.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSim(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                    g2oCorrectedSiw = g2oSim*mg2oMergeScw;
                    vCorrectedSim3[pKFi]=g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                    Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                    double s = g2oCorrectedSiw.scale();

                    pKFi->mfScale = s;
                    eigt *=(1./s); //[R t/s;0 1]

                    cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

                    //pKFi->mTcwBefMerge = pKFi->GetPose();
                    //pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                    pKFi->GetPose().copyTo(pKFi->mTcwBefMerge);
                    pKFi->GetPoseInverse().copyTo(pKFi->mTwcBefMerge);

                    pKFi->SetPose(correctedTiw);

                    if(pCurrentMap->isImuInitialized())
                    {
                        Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
                        pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity()); // TODO: should add here scale s
                    }

                }
                for(boost::interprocess::offset_ptr<MapPoint>  pMPi : vpCurrentMapMPs)
                {
                    if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
                        continue;

                    boost::interprocess::offset_ptr<KeyFrame>  pKFref = pMPi->GetReferenceKeyFrame();
                    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                    // Project with non-corrected pose and project back with corrected pose
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                    Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));

                    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                    pMPi->SetWorldPos(cvCorrectedP3Dw);

                    pMPi->UpdateNormalAndDepth();
                }
            }
        }
        std::cout<<"Merge local 10\n";

        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // Optimize graph (and update the loop position for each element form the begining to the end)
        if(mpTracker->mSensor != System::MONOCULAR)
        {
            Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
        }


        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpCurrentMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }

            for(boost::interprocess::offset_ptr<MapPoint>  pMPi : vpCurrentMapMPs)
            {
                if(!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
        }
    }

    mpLocalMapper->Release();
    std::cout<<"Merge local 11\n";

    Verbose::PrintMess("MERGE:Completed!!!!!", Verbose::VERBOSITY_DEBUG);

    if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
    {
        // Launch a new thread to perform Global Bundle Adjustment
        Verbose::PrintMess("Relaunch Global BA", Verbose::VERBOSITY_DEBUG);
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
    }

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();
    pMergeMap->IncreaseChangeIndex();

    mpAtlas->RemoveBadMaps();

}

void LoopClosing::MergeLocal2()
{
    cout << "Merge detected!!!!" << endl;

    int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    boost::interprocess::offset_ptr<KeyFrame>  pNewChild;
    boost::interprocess::offset_ptr<KeyFrame>  pNewParent;

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpLocalCurrentWindowKFs;
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    cout << "Check Full Bundle Adjustment" << endl;
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }


    cout << "Request Stop Local Mapping" << endl;
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    cout << "Local Map stopped" << endl;

    boost::interprocess::offset_ptr<Map>  pCurrentMap = mpCurrentKF->GetMap();
    boost::interprocess::offset_ptr<Map>  pMergeMap = mpMergeMatchedKF->GetMap();

    {
        float s_on = mSold_new.scale();
        cv::Mat R_on = Converter::toCvMat(mSold_new.rotation().toRotationMatrix());
        cv::Mat t_on = Converter::toCvMat(mSold_new.translation());

        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

        mpLocalMapper->EmptyQueue();

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        bool bScaleVel=false;
        if(s_on!=1)
            bScaleVel=true;
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(R_on,s_on,bScaleVel,t_on);
        mpTracker->UpdateFrameIMU(s_on,mpCurrentKF->GetImuBias(),mpTracker->GetLastKeyFrame());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    }

    const int numKFnew=pCurrentMap->KeyFramesInMap();

    if((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)&& !pCurrentMap->GetIniertialBA2()){
        // Map is not completly initialized
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap,bg,ba);
        IMU::Bias b (ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        mpTracker->UpdateFrameIMU(1.0f,b,mpTracker->GetLastKeyFrame());

        // Set map initialized
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();

    }

    // Load KFs and MPs from merge map
    {
        // Get Merge Map Mutex (This section stops tracking!!)
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map


        vector<boost::interprocess::offset_ptr<KeyFrame> > vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
        vector<boost::interprocess::offset_ptr<MapPoint> > vpMergeMapMPs = pMergeMap->GetAllMapPoints();


        for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpMergeMapKFs)
        {
            if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        for(boost::interprocess::offset_ptr<MapPoint>  pMPi : vpMergeMapMPs)
        {
            if(!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        // Save non corrected poses (already merged maps)
        vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = pCurrentMap->GetAllKeyFrames();
        for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpKFs)
        {
            cv::Mat Tiw=pKFi->GetPose();
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            NonCorrectedSim3[pKFi]=g2oSiw;
        }
    }

    pMergeMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);
    while(pNewChild)
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        boost::interprocess::offset_ptr<KeyFrame>  pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }

    vector<boost::interprocess::offset_ptr<MapPoint> > vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
    vector<boost::interprocess::offset_ptr<KeyFrame> > vpCurrentConnectedKFs;

    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<boost::interprocess::offset_ptr<KeyFrame> > aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mvpMergeConnectedKFs.size()>6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin()+6,mvpMergeConnectedKFs.end());

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
    if (vpCurrentConnectedKFs.size()>6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin()+6,vpCurrentConnectedKFs.end());

    set<boost::interprocess::offset_ptr<MapPoint> > spMapPointMerge;
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : mvpMergeConnectedKFs)
    {
        set<boost::interprocess::offset_ptr<MapPoint> > vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        if(spMapPointMerge.size()>1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : vpCurrentConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(boost::interprocess::offset_ptr<KeyFrame>  pKFi : mvpMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    if (numKFnew<10){
        mpLocalMapper->Release();
        return;
    }

    // Perform BA
    bool bStopFlag=false;
    boost::interprocess::offset_ptr<KeyFrame>  pCurrKF = mpTracker->GetLastKeyFrame();
    Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,CorrectedSim3);

    // Release Local Mapping.
    mpLocalMapper->Release();


    return;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        boost::interprocess::offset_ptr<KeyFrame>  pKFi = mit->first;
        boost::interprocess::offset_ptr<Map>  pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<boost::interprocess::offset_ptr<MapPoint> > vpReplacePoints(vpMapPoints.size(),static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
        int numFused = matcher.Fuse(pKFi,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            boost::interprocess::offset_ptr<MapPoint>  pRep = vpReplacePoints[i];
            if(pRep)
            {


                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);

            }
        }

        total_replaces += num_replaces;
    }
}


void LoopClosing::SearchAndFuse(const vector<boost::interprocess::offset_ptr<KeyFrame> > &vConectedKFs, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for(auto mit=vConectedKFs.begin(), mend=vConectedKFs.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        boost::interprocess::offset_ptr<KeyFrame>  pKF = (*mit);
        boost::interprocess::offset_ptr<Map>  pMap = pKF->GetMap();
        cv::Mat cvScw = pKF->GetPose();

        vector<boost::interprocess::offset_ptr<MapPoint> > vpReplacePoints(vpMapPoints.size(),static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL));
        matcher.Fuse(pKF,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            boost::interprocess::offset_ptr<MapPoint>  pRep = vpReplacePoints[i];
            if(pRep)
            {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        total_replaces += num_replaces;
    }
}



void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(boost::interprocess::offset_ptr<Map> pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
        mbResetActiveMapRequested = false;
    }
    else if(mbResetActiveMapRequested)
    {

        for (list<boost::interprocess::offset_ptr<KeyFrame> >::const_iterator it=mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
        {
            boost::interprocess::offset_ptr<KeyFrame>  pKFi = *it;
            if(pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid=mpAtlas->GetLastInitKFid();
        mbResetActiveMapRequested=false;

    }
}

void LoopClosing::RunGlobalBundleAdjustment(boost::interprocess::offset_ptr<Map>  pActiveMap, unsigned long nLoopKF)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

    const bool bImuInit = pActiveMap->isImuInitialized();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();
#endif

    if(!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
    else
        Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartMapUpdate = std::chrono::steady_clock::now();

    double timeFullGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartMapUpdate - time_StartFGBA).count();
    vTimeFullGBA_ms.push_back(timeFullGBA);
#endif


    int idx =  mnFullBAIdx;

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!bImuInit && pActiveMap->isImuInitialized())
            return;

        if(!mbStopGBA)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            //old
            //list<boost::interprocess::offset_ptr<KeyFrame> > lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());
            //new
            list<boost::interprocess::offset_ptr<KeyFrame> > lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins->begin(),pActiveMap->mvpKeyFrameOrigins->end());

            while(!lpKFtoCheck.empty())
            {
                boost::interprocess::offset_ptr<KeyFrame>  pKF = lpKFtoCheck.front();
                const set<boost::interprocess::offset_ptr<KeyFrame> > sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<boost::interprocess::offset_ptr<KeyFrame> >::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    boost::interprocess::offset_ptr<KeyFrame>  pChild = *sit;
                    if(!pChild || pChild->isBad())
                        continue;

                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        //pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;
                        cv::Mat temp_mat1 = Tchildc*pKF->mTcwGBA;
                        (temp_mat1).copyTo(pChild->mTcwGBA);

                        cv::Mat Rcor = pChild->mTcwGBA.rowRange(0,3).colRange(0,3).t()*pChild->GetRotation();
                        if(!pChild->GetVelocity().empty()){
                            //pChild->mVwbGBA = Rcor*pChild->GetVelocity();
                            cv::Mat temp_mat2 = Rcor*pChild->GetVelocity();
                            (temp_mat2).copyTo(pChild->mVwbGBA);
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                        pChild->mBiasGBA = pChild->GetImuBias();


                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                //pKF->mTcwBefGBA = pKF->GetPose();
                pKF->GetPose().copyTo(pKF->mTcwBefGBA);
                pKF->SetPose(pKF->mTcwGBA);

                if(pKF->bImu)
                {
                    //pKF->mVwbBefGBA = pKF->GetVelocity();
                    pKF->GetVelocity().copyTo(pKF->mVwbBefGBA);
                    if (pKF->mVwbGBA.empty())
                        Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                    assert(!pKF->mVwbGBA.empty());
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);                    
                }

                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<boost::interprocess::offset_ptr<MapPoint> > vpMPs = pActiveMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                boost::interprocess::offset_ptr<MapPoint>  pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    boost::interprocess::offset_ptr<KeyFrame>  pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    if(pRefKF->mTcwBefGBA.empty())
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            mpLocalMapper->Release();

            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndMapUpdate = std::chrono::steady_clock::now();

    double timeMapUpdate = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMapUpdate - time_StartMapUpdate).count();
    vTimeMapUpdate_ms.push_back(timeMapUpdate);

    double timeGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMapUpdate - time_StartFGBA).count();
    vTimeGBATotal_ms.push_back(timeGBA);
#endif
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
