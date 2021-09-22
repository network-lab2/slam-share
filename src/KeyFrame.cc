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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "ImuTypes.h"
#include "System.h"
#include<mutex>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), /*mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),*/
        /*mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)),*/ /*mDescriptors(NULL),*/
        /*mBowVec(NULL), mFeatVec(NULL),*/ mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0),/* mvScaleFactors(0), mvLevelSigma2(0),
        mvInvLevelSigma2(0),*/ mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), /*mK(NULL),*/  mPrevKF(static_cast<boost::interprocess::offset_ptr<KeyFrame> >(NULL)), mNextKF(static_cast<boost::interprocess::offset_ptr<KeyFrame> >(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mbHasHessian(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0)
{
    //creating all the matrix in the keyframe
    std::cout<<"Keyframe constructor.--++ First: initializing all vectors, sets and CV::MATs."<<std::endl;

    //constructor arguments
    (*mvKeys).clear();// = (static_cast<vector<cv::KeyPoint> >(NULL));
    (*mvKeysUn).clear();

    //allocators first.
    const ShmemAllocator_mappoint alloc_inst(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_keyframe alloc_inst_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_map_keyframe alloc_map_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_keyframe_set alloc_set_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_cv_keypoint alloc_set_cv(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_float alloc_set_float(ORB_SLAM3::segment.get_segment_manager());
    //An allocator convertible to any allocator<T, segment_manager_t> type
    const void_allocator alloc_inst_void (ORB_SLAM3::segment.get_segment_manager());

    //keyframes ones
    mvpLoopCandKFs = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);
    mvpMergeCandKFs = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);
    mvpOrderedConnectedKeyFrames = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);

    //mappoints ones
    mvpMapPoints = ORB_SLAM3::segment.construct<MyVector_mappoint>(boost::interprocess::anonymous_instance)(alloc_inst);
    //mvpMapPoints = mvpMapPoints_original;

    //map-keyframe
    mConnectedKeyFrameWeights = ORB_SLAM3::segment.construct<MyMap>(boost::interprocess::anonymous_instance)(std::less<boost::interprocess::offset_ptr<KeyFrame> >(),alloc_map_key);

    //set-keyframe
    mspMergeEdges = ORB_SLAM3::segment.construct<Myset_keyframe>(boost::interprocess::anonymous_instance)(alloc_set_key);

    //mvkeys and mvkeysun
    mvKeys = ORB_SLAM3::segment.construct<MyVector_CV>(boost::interprocess::anonymous_instance)(alloc_set_cv);
    mvKeysUn = ORB_SLAM3::segment.construct<MyVector_CV>(boost::interprocess::anonymous_instance)(alloc_set_cv);

    //constructor arguments
    (*mvKeys).clear();// = (static_cast<vector<cv::KeyPoint> >(NULL));
    (*mvKeysUn).clear();

    //float vectors
    mvuRight = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvDepth = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvScaleFactors = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvLevelSigma2 = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvInvLevelSigma2 = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvuRight->clear();
    mvDepth->clear();
    mvScaleFactors->clear();
    mvLevelSigma2->clear();
    mvInvLevelSigma2->clear();

    /* the triple vector */
    mGridRight = ORB_SLAM3::segment.construct<size_t_vector_vector_vector>(boost::interprocess::anonymous_instance)(alloc_inst_void);
    mGrid = ORB_SLAM3::segment.construct<size_t_vector_vector_vector>(boost::interprocess::anonymous_instance)(alloc_inst_void);


}

KeyFrame::KeyFrame(Frame &F, boost::interprocess::offset_ptr<Map> pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), /*mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),*/
    /*mvuRight(F.mvuRight), mvDepth(F.mvDepth),*/ mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), /*mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2),*/ mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib)/*, mvpMapPoints(F.mvpMapPoints)*/, mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mbHasHessian(false), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    /*mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch),*/mTlr(F.mTlr.clone()),
    /*mvKeysRight(F.mvKeysRight),*/ NLeft(F.Nleft), NRight(F.Nright), mTrl(F.mTrl), mnNumberOfOpt(0)
{

    //(*mvKeys).assign(F.mvKeys.begin(), F.mvKeys.end());
    //(*mvKeysUn).assign(F.mvKeysUn.begin(), F.mvKeysUn.end());
     //An allocator convertible to any allocator<T, segment_manager_t> type
    const void_allocator alloc_inst_void (ORB_SLAM3::segment.get_segment_manager());
    const size_t_allocator alloc_size_t (ORB_SLAM3::segment.get_segment_manager());
    const size_t_vector_allocator alloc_size_t_vector(ORB_SLAM3::segment.get_segment_manager());
    const size_t_vector_vector_allocator alloc_size_t_vector_vector (ORB_SLAM3::segment.get_segment_manager());
     /* the triple vector */
    mGridRight = ORB_SLAM3::segment.construct<size_t_vector_vector_vector>(boost::interprocess::anonymous_instance)(alloc_size_t_vector_vector);
    mGrid = ORB_SLAM3::segment.construct<size_t_vector_vector_vector>(boost::interprocess::anonymous_instance)(alloc_inst_void);
    
    imgLeft = F.imgLeft.clone();
    imgRight = F.imgRight.clone();

    mnId=nNextId++;

    std::cout<<"Keyframe constructor: mnID "<<mnId<<std::endl;

    mGrid->reserve(mnGridCols);//mGrid.resize(mnGridCols);
    mGrid->push_back(F.mGrid[0]);
    mGridRight->reserve(mnGridCols);
    std::cout<<"Reserve complete\n";
    //if(F.Nleft != -1)  (mGridRight.get())->resize(mnGridCols);//if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid->at(i).reserve(mnGridRows);//mGrid[i].resize(mnGridRows);
        std::cout<<"Inside loop after reserve."<<std::endl;
        if(F.Nleft != -1) mGridRight->at(i).reserve(mnGridRows);//if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            (mGrid->at(i)).at(j).assign(F.mGrid[i][j].begin(),F.mGrid[i][j].end());//mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                (mGridRight->at(i)).at(j).assign(F.mGridRight[i][j].begin(),F.mGridRight[i][j].end());//mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }

    //initializing a vector
    const ShmemAllocator_mappoint alloc_inst(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_keyframe alloc_inst_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_map_keyframe alloc_map_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_keyframe_set alloc_set_key(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_cv_keypoint alloc_set_cv(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_float alloc_set_float(ORB_SLAM3::segment.get_segment_manager());
    const ShmemAllocator_int alloc_set_int(ORB_SLAM3::segment.get_segment_manager());

   
    


    //keyframes ones
    mvpLoopCandKFs = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);
    mvpMergeCandKFs = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);
    mvpOrderedConnectedKeyFrames = ORB_SLAM3::segment.construct<MyVector_keyframe>(boost::interprocess::anonymous_instance)(alloc_inst_key);

    //mappoints ones
    mvpMapPoints = ORB_SLAM3::segment.construct<MyVector_mappoint>(boost::interprocess::anonymous_instance)(alloc_inst);

     //map-keyframe
    mConnectedKeyFrameWeights = ORB_SLAM3::segment.construct<MyMap>(boost::interprocess::anonymous_instance)(std::less<boost::interprocess::offset_ptr<KeyFrame> >(),alloc_map_key);


    //set-keyframe
    mspMergeEdges = ORB_SLAM3::segment.construct<Myset_keyframe>(boost::interprocess::anonymous_instance)(alloc_set_key);

    //mvpMapPoints = mvpMapPoints_original;
    (*mvpMapPoints).assign(F.mvpMapPoints.begin(), F.mvpMapPoints.end());

    //mvkeys and mvkeysun cv vectors
    mvKeys = ORB_SLAM3::segment.construct<MyVector_CV>(boost::interprocess::anonymous_instance)(alloc_set_cv);
    mvKeysUn = ORB_SLAM3::segment.construct<MyVector_CV>(boost::interprocess::anonymous_instance)(alloc_set_cv);
    mvKeysRight = ORB_SLAM3::segment.construct<MyVector_CV>(boost::interprocess::anonymous_instance)(alloc_set_cv);


    (mvKeys)->assign(F.mvKeys.begin(), F.mvKeys.end());
    (mvKeysUn)->assign(F.mvKeysUn.begin(), F.mvKeysUn.end());
    mvKeysRight->assign(F.mvKeysRight.begin(),F.mvKeysRight.end());

    //float vectors
    mvuRight = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvDepth = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvScaleFactors = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvLevelSigma2 = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvInvLevelSigma2 = ORB_SLAM3::segment.construct<MyVector_float>(boost::interprocess::anonymous_instance)(alloc_set_float);
    mvuRight->assign(F.mvuRight.begin(),F.mvuRight.end());
    mvDepth->assign(F.mvDepth.begin(), F.mvDepth.end());
    mvScaleFactors->assign(F.mvScaleFactors.begin(),F.mvScaleFactors.end());
    mvLevelSigma2->assign(F.mvLevelSigma2.begin(),F.mvLevelSigma2.end());
    mvInvLevelSigma2->assign(F.mvInvLevelSigma2.begin(),F.mvInvLevelSigma2.end());

    //int vectors
    mvLeftToRightMatch = ORB_SLAM3::segment.construct<MyVector_int>(boost::interprocess::anonymous_instance)(alloc_set_int);
    mvRightToLeftMatch = ORB_SLAM3::segment.construct<MyVector_int>(boost::interprocess::anonymous_instance)(alloc_set_int);
    mvLeftToRightMatch->assign(F.mvLeftToRightMatch.begin(),F.mvLeftToRightMatch.end());
    mvRightToLeftMatch->assign(F.mvRightToLeftMatch.begin(),F.mvRightToLeftMatch.end());

   

    //creating all the matrix in the keyframe
    std::cout<<"Keyframe constructor.--++ this one is used"<<std::endl;
    mTcwGBA_ptr = ORB_SLAM3::allocator_instance.allocate(10*4*4);
    mTcwBefGBA_ptr = ORB_SLAM3::allocator_instance.allocate(10*4*4);
    mVwbGBA_ptr = ORB_SLAM3::allocator_instance.allocate(10*4*4);
    mVwbBefGBA_ptr = ORB_SLAM3::allocator_instance.allocate(10*4*4);

    //for basic matrices.
    Tcw_ptr = ORB_SLAM3::allocator_instance.allocate(4*4*4);
    Twc_ptr = ORB_SLAM3::allocator_instance.allocate(4*4*4);
    Ow_ptr = ORB_SLAM3::allocator_instance.allocate(3*1*4);
    Cw_ptr = ORB_SLAM3::allocator_instance.allocate(4*4);
    Owb_ptr = ORB_SLAM3::allocator_instance.allocate(4*4);
    Vw_ptr = ORB_SLAM3::allocator_instance.allocate(3*4);

    Tcw = cv::Mat(4,4,CV_32F,Tcw_ptr.get());
    Twc = cv::Mat(4,4,CV_32F,Twc_ptr.get());
    Ow = cv::Mat(3,1,CV_32F,Ow_ptr.get());
    Cw = cv::Mat(4,1,CV_32F,Cw_ptr.get());
    Owb = cv::Mat(3,1,CV_32F,Owb_ptr.get());


    if(F.mVw.empty())
        Vw = cv::Mat::zeros(3,1,CV_32F);
    else
        Vw = F.mVw.clone();

    mImuBias = F.mImuBias;
    SetPose(F.mTcw);

    mnOriginMapId = pMap->GetId();

    this->Tlr_ = cv::Matx44f(mTlr.at<float>(0,0),mTlr.at<float>(0,1),mTlr.at<float>(0,2),mTlr.at<float>(0,3),
                             mTlr.at<float>(1,0),mTlr.at<float>(1,1),mTlr.at<float>(1,2),mTlr.at<float>(1,3),
                             mTlr.at<float>(2,0),mTlr.at<float>(2,1),mTlr.at<float>(2,2),mTlr.at<float>(2,3),
                             mTlr.at<float>(3,0),mTlr.at<float>(3,1),mTlr.at<float>(3,2),mTlr.at<float>(3,3));

}
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;
    if (!mImuCalib.Tcb.empty())
        Owb = Rwc*mImuCalib.Tcb.rowRange(0,3).col(3)+Ow;


    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;

    //Static matrices
    this->Tcw_ = cv::Matx44f(Tcw.at<float>(0,0),Tcw.at<float>(0,1),Tcw.at<float>(0,2),Tcw.at<float>(0,3),
                       Tcw.at<float>(1,0),Tcw.at<float>(1,1),Tcw.at<float>(1,2),Tcw.at<float>(1,3),
                       Tcw.at<float>(2,0),Tcw.at<float>(2,1),Tcw.at<float>(2,2),Tcw.at<float>(2,3),
                       Tcw.at<float>(3,0),Tcw.at<float>(3,1),Tcw.at<float>(3,2),Tcw.at<float>(3,3));

    this->Twc_ = cv::Matx44f(Twc.at<float>(0,0),Twc.at<float>(0,1),Twc.at<float>(0,2),Twc.at<float>(0,3),
                             Twc.at<float>(1,0),Twc.at<float>(1,1),Twc.at<float>(1,2),Twc.at<float>(1,3),
                             Twc.at<float>(2,0),Twc.at<float>(2,1),Twc.at<float>(2,2),Twc.at<float>(2,3),
                                     Twc.at<float>(3,0),Twc.at<float>(3,1),Twc.at<float>(3,2),Twc.at<float>(3,3));

    this->Ow_ = cv::Matx31f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
}

void KeyFrame::SetVelocity(const cv::Mat &Vw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Vw_.copyTo(Vw);
}


cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return Owb.clone();
}

cv::Mat KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0,3).colRange(0,3)*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
}

cv::Mat KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc*mImuCalib.Tcb;
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return Vw.clone();
}

void KeyFrame::AddConnection(boost::interprocess::offset_ptr<KeyFrame> pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        //old-code
        /*
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
        */
        //new-code
        if(!(*mConnectedKeyFrameWeights).count(pKF))
            (*mConnectedKeyFrameWeights)[pKF]=weight;
        else if ((*mConnectedKeyFrameWeights)[pKF]!=weight)
            (*mConnectedKeyFrameWeights)[pKF]=weight;
        else
            return;

    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,boost::interprocess::offset_ptr<KeyFrame> > > vPairs;
    
    //vPairs.reserve(mConnectedKeyFrameWeights.size());//old-code
    vPairs.reserve((*mConnectedKeyFrameWeights).size());
    
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++) //old-code
    //new-code
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit=mConnectedKeyFrameWeights->begin(), mend=mConnectedKeyFrameWeights->end(); mit!=mend; mit++)
     for(auto const& mit : (*mConnectedKeyFrameWeights)){
       //vPairs.push_back(make_pair(mit->second,mit->first));
        vPairs.push_back(make_pair(mit.second,mit.first));
   }

    sort(vPairs.begin(),vPairs.end());
    list<boost::interprocess::offset_ptr<KeyFrame> > lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    //old code
    //mvpOrderedConnectedKeyFrames = vector<boost::interprocess::offset_ptr<KeyFrame> >(lKFs.begin(),lKFs.end());
    //new code
    mvpOrderedConnectedKeyFrames->assign(lKFs.begin(),lKFs.end());

    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<boost::interprocess::offset_ptr<KeyFrame> > s;
    //old-code
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
    //new-code
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit=mConnectedKeyFrameWeights->begin();mit!=mConnectedKeyFrameWeights->end();mit++)
    for(auto const& mit : (*mConnectedKeyFrameWeights)){
        s.insert(mit.first);
    }
    return s;
}

vector<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    //old-code
    //return mvpOrderedConnectedKeyFrames;
    //new-code
    return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames->begin(),mvpOrderedConnectedKeyFrames->end());
}

vector<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    std::cout<<"GetBestCovisibilityKeyFrames before IF\n value of size: "<<(*mvpOrderedConnectedKeyFrames).size()<<std::endl;
    //old-code
    //if((int)mvpOrderedConnectedKeyFrames.size()<N){
    //new code
    if(((int)(mvpOrderedConnectedKeyFrames->size()))<N){
        std::cout<<"GetBestCovisibilityKeyFrames if (true)\n";
        //old-code
        //return mvpOrderedConnectedKeyFrames;
        //new-code
        return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames->begin(), mvpOrderedConnectedKeyFrames->end());

    }
    else{
        std::cout<<"GetBestCovisibilityKeyFrames if (false)\n";
        //old-code
        //return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);
        //new-code
        return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames->begin(),mvpOrderedConnectedKeyFrames->begin()+N);
    }

}

vector<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    //old-code
    //if(mvpOrderedConnectedKeyFrames.empty())
    //new-code
    if(mvpOrderedConnectedKeyFrames->empty())
    {
        return vector<boost::interprocess::offset_ptr<KeyFrame> >();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<boost::interprocess::offset_ptr<KeyFrame> >();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        //old-code
        //return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
        //new-code
        return vector<boost::interprocess::offset_ptr<KeyFrame> >(mvpOrderedConnectedKeyFrames->begin(), mvpOrderedConnectedKeyFrames->begin()+n);
    }
}

int KeyFrame::GetWeight(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    /* //old code
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
        */
    //new-code
    if(mConnectedKeyFrameWeights->count(pKF))
        return mConnectedKeyFrameWeights->at(pKF);
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    //for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    std::cout<<"GetNumberMps, before loop\n";
    //boost::interprocess::offset_ptr<MyVector_mappoint> mvpMapPoints = mvpMapPoints_ptr;
    for(size_t i=0, iend=(mvpMapPoints)->size(); i<iend; i++)
    {
        //if(!mvpMapPoints[i])
        if(!(mvpMapPoints->at(i)))
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(boost::interprocess::offset_ptr<MapPoint> pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //mvpMapPoints[idx]=pMP;
    (*mvpMapPoints).at(idx)=pMP;
    //mvpMapPoints->at(idx)=pMP;

    //Let's check all the matrix sizes of mappoint here:
    /*
    std::cout<<" Name      "<<"Rows X Cols"<<" bytes "<<std::endl;
    std::cout<<"mWorldPos: "<<pMP->GetWorldPos().size<<" "<<pMP->GetWorldPos().total()*pMP->GetWorldPos().elemSize()<<std::endl;
    std::cout<<"mNormalVector: "<<pMP->GetNormal().size<<" "<<pMP->GetNormal().total()*pMP->GetNormal().elemSize()<<std::endl;
    std::cout<<"mPosGBA: "<<pMP->mPosGBA.size<<" "<<pMP->mPosGBA.total()*pMP->mPosGBA.elemSize()<<std::endl;
    std::cout<<"mPosMerge: "<<pMP->mPosMerge.size<<" "<<pMP->mPosMerge.total()*pMP->mPosMerge.elemSize()<<std::endl;
    std::cout<<"mNormalVectorMerge: "<<pMP->mNormalVectorMerge.size<<" "<<pMP->mNormalVectorMerge.total()*pMP->mNormalVectorMerge.elemSize()<<std::endl;
    */

    //std::cout<<"mWorldPosx: "<<pMP->GetWorldPos2().size<<pMP->GetWorldPos2().total()*pMP->GetWorldPos2().elemSize()<<std::endl;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //mvpMapPoints[idx]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
    (*mvpMapPoints)[idx]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
}

void KeyFrame::EraseMapPointMatch(boost::interprocess::offset_ptr<MapPoint>  pMP)
{
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1){
        //mvpMapPoints[leftIndex]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
        (*mvpMapPoints)[leftIndex]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
    }
    if(rightIndex != -1){
        //mvpMapPoints[rightIndex]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
        (*mvpMapPoints)[rightIndex]=static_cast<boost::interprocess::offset_ptr<MapPoint> >(NULL);
    }
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, boost::interprocess::offset_ptr<MapPoint>  pMP)
{
    //mvpMapPoints[idx]=pMP;
    (*mvpMapPoints)[idx]=pMP;
}

set<boost::interprocess::offset_ptr<MapPoint> > KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<boost::interprocess::offset_ptr<MapPoint> > s;
    //for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    for(size_t i=0, iend=mvpMapPoints->size(); i<iend; i++)
    {
        //if(!mvpMapPoints[i]){
        if(!(*mvpMapPoints)[i]){
            continue;
        }
        //boost::interprocess::offset_ptr<MapPoint>  pMP = mvpMapPoints[i];
        boost::interprocess::offset_ptr<MapPoint>  pMP = (*mvpMapPoints)[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        //boost::interprocess::offset_ptr<MapPoint>  pMP = mvpMapPoints[i];
        boost::interprocess::offset_ptr<MapPoint>  pMP = (*mvpMapPoints)[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    //if(mvpMapPoints[i]->Observations()>=minObs)
                    if((*mvpMapPoints)[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<boost::interprocess::offset_ptr<MapPoint> > KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    //return mvpMapPoints;
    
    mvpMapPoints_vector.clear();
    /*
    for(size_t i=0; i<mvpMapPoints->size(); i++){
        mvpMapPoints_vector.push_back((*mvpMapPoints)[i]);
    }
    */
    mvpMapPoints_vector.assign(mvpMapPoints.get()->begin(),mvpMapPoints.get()->end());
    //return (*(mvpMapPoints.get()));
    return mvpMapPoints_vector;
}

boost::interprocess::offset_ptr<MapPoint>  KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //return mvpMapPoints[idx];
    return (*mvpMapPoints)[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    map<boost::interprocess::offset_ptr<KeyFrame> ,int> KFcounter;

    vector<boost::interprocess::offset_ptr<MapPoint> > vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        //vpMP = mvpMapPoints;
        //vpMP = (*mvpMapPoints);
        mvpMapPoints_vector.clear();
    for(size_t i=0; i<mvpMapPoints->size(); i++){
        mvpMapPoints_vector.push_back((*mvpMapPoints)[i]);
    }
        vpMP = mvpMapPoints_vector;

    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<boost::interprocess::offset_ptr<MapPoint> >::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        boost::interprocess::offset_ptr<MapPoint>  pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<boost::interprocess::offset_ptr<KeyFrame> ,tuple<int,int>> observations = pMP->GetObservations();

        for(map<boost::interprocess::offset_ptr<KeyFrame> ,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;

        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    boost::interprocess::offset_ptr<KeyFrame>  pKFmax=NULL;
    int th = 15;

    vector<pair<int,boost::interprocess::offset_ptr<KeyFrame> > > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<boost::interprocess::offset_ptr<KeyFrame> > lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        //turned off for check
        //mConnectedKeyFrameWeights = KFcounter;
        mConnectedKeyFrameWeights->insert(KFcounter.begin(),KFcounter.end());
        // old-code
        //mvpOrderedConnectedKeyFrames = vector<boost::interprocess::offset_ptr<KeyFrame> >(lKFs.begin(),lKFs.end());
        // new-code
        mvpOrderedConnectedKeyFrames->assign(lKFs.begin(),lKFs.end());

        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            //old-code
            //mpParent = mvpOrderedConnectedKeyFrames.front();
            //new-code
            mpParent = mvpOrderedConnectedKeyFrames->front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(this);
}

set<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

boost::interprocess::offset_ptr<KeyFrame>  KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(boost::interprocess::offset_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(boost::interprocess::offset_ptr<KeyFrame>  pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    //old code
    //mspMergeEdges.insert(pKF);
    //new code
    mspMergeEdges->insert(pKF);
}

set<boost::interprocess::offset_ptr<KeyFrame> > KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    //clear the set and then initialize it with real value
    //new code
    mspMergeEdges_support.clear();

/*
    for(auto f : mspMergeEdges){
        mspMergeEdges_support.insert(f);
    }
    */
    //code ends

    //old code
    //return mspMergeEdges;
    //new code
    return mspMergeEdges_support;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    //old-code
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    //new-code
    //for(map<boost::interprocess::offset_ptr<KeyFrame> ,int>::iterator mit = mConnectedKeyFrameWeights->begin(), mend=mConnectedKeyFrameWeights->end(); mit!=mend; mit++)
    for(auto const& mit : (*mConnectedKeyFrameWeights))// = mConnectedKeyFrameWeights->begin(), mend=mConnectedKeyFrameWeights->end(); mit!=mend; mit++)
    {
        //mit->first->EraseConnection(this);
        mit.first->EraseConnection(this);
    }

    //for(size_t i=0; i<mvpMapPoints.size(); i++)
    for(size_t i=0; i<mvpMapPoints->size(); i++)
    {
        //if(mvpMapPoints[i])
        if((*mvpMapPoints)[i])
        {
            //mvpMapPoints[i]->EraseObservation(this);
            (*mvpMapPoints)[i]->EraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        //old-code
        //mConnectedKeyFrameWeights.clear();
        //new-code
        mConnectedKeyFrameWeights->clear();
        //old-code
        //mvpOrderedConnectedKeyFrames.clear();
        //new-code
        mvpOrderedConnectedKeyFrames->clear();

        // Update Spanning Tree
        set<boost::interprocess::offset_ptr<KeyFrame> > sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            boost::interprocess::offset_ptr<KeyFrame>  pC;
            boost::interprocess::offset_ptr<KeyFrame>  pP;

            for(set<boost::interprocess::offset_ptr<KeyFrame> >::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                boost::interprocess::offset_ptr<KeyFrame>  pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<boost::interprocess::offset_ptr<KeyFrame> > vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<boost::interprocess::offset_ptr<KeyFrame> >::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<boost::interprocess::offset_ptr<KeyFrame> >::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(this);
            mTcp = Tcw*mpParent->GetPoseInverse();
        }
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(boost::interprocess::offset_ptr<KeyFrame>  pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        //if(mConnectedKeyFrameWeights.count(pKF)) //old-code
        if(mConnectedKeyFrameWeights->count(pKF))
        {
            //mConnectedKeyFrameWeights.erase(pKF);
            mConnectedKeyFrameWeights->erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);
    std::vector<size_t> buffer_mGridRight;
    std::vector<size_t> buffer_mGrid;


    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            //copy the mgrid vector to return
            std::cout<<"Copy the mgrid vector to return"<<std::endl;
            buffer_mGridRight.clear();
            buffer_mGrid.clear();
            if(bRight)
                buffer_mGridRight.assign((mGridRight->at(ix)).at(iy).begin(),(mGridRight->at(ix)).at(iy).end());
            buffer_mGrid.assign((mGrid->at(ix)).at(iy).begin(),(mGrid->at(ix)).at(iy).end());
            const vector<size_t> vCell = (!bRight) ? buffer_mGrid : buffer_mGridRight;//const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            std::cout<<"vCell size: "<<vCell.size()<<std::endl;
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? (*mvKeysUn)[vCell[j]]
                                                         : (!bRight) ? (*mvKeys)[vCell[j]]
                                                                     : (*mvKeysRight)[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth->at(i);//const float z = mvDepth[i];
    if(z>0)
    {
        const float u = (*mvKeys)[i].pt.x;
        const float v = (*mvKeys)[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<boost::interprocess::offset_ptr<MapPoint> > vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints.assign(mvpMapPoints.get()->begin(),mvpMapPoints.get()->end());// = mvpMapPoints;
        /*
        mvpMapPoints_vector.clear();
        for(size_t i=0; i<mvpMapPoints->size(); i++){
            mvpMapPoints_vector.push_back((*mvpMapPoints)[i]);
        }
        vpMapPoints = mvpMapPoints_vector;
        */
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        //if(mvpMapPoints[i])
        if((*mvpMapPoints)[i])
        {
            //boost::interprocess::offset_ptr<MapPoint>  pMP = mvpMapPoints[i];
            boost::interprocess::offset_ptr<MapPoint>  pMP = (*mvpMapPoints)[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

cv::Mat KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

cv::Mat KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

boost::interprocess::offset_ptr<Map>  KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(boost::interprocess::offset_ptr<Map>  pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

bool KeyFrame::ProjectPointDistort(boost::interprocess::offset_ptr<MapPoint>  pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);

    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(boost::interprocess::offset_ptr<MapPoint>  pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

cv::Mat KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    cv::Mat Trw;
    cv::hconcat(Rrw,trw,Trw);

    return Trw.clone();
}

cv::Mat KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = GetCameraCenter();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightPoseInverseH() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);
    cv::Mat h(1,4,CV_32F,cv::Scalar(0.0f)); h.at<float>(3) = 1.0f;
    cv::vconcat(Twr,h,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    return twr.clone();
}

cv::Mat KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    return Rrw.clone();

}

cv::Mat KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    return trw.clone();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

cv::Matx33f KeyFrame::GetRotation_() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_.get_minor<3,3>(0,0);
}

cv::Matx31f KeyFrame::GetTranslation_() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_.get_minor<3,1>(0,3);
}

cv::Matx31f KeyFrame::GetCameraCenter_() {
    unique_lock<mutex> lock(mMutexPose);
    return Ow_;
}

cv::Matx33f KeyFrame::GetRightRotation_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx33f Rlw = Tcw_.get_minor<3,3>(0,0);
    cv::Matx33f Rrw = Rrl * Rlw;

    return Rrw;
}

cv::Matx31f KeyFrame::GetRightTranslation_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx31f tlw = Tcw_.get_minor<3,1>(0,3);
    cv::Matx31f trl = - Rrl * Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f trw = Rrl * tlw + trl;

    return trw;
}

cv::Matx44f KeyFrame::GetRightPose_() {
    unique_lock<mutex> lock(mMutexPose);

    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx33f Rlw = Tcw_.get_minor<3,3>(0,0);
    cv::Matx33f Rrw = Rrl * Rlw;

    cv::Matx31f tlw = Tcw_.get_minor<3,1>(0,3);
    cv::Matx31f trl = - Rrl * Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f trw = Rrl * tlw + trl;

    cv::Matx44f Trw{Rrw(0,0),Rrw(0,1),Rrw(0,2),trw(0),
                    Rrw(1,0),Rrw(1,1),Rrw(1,2),trw(1),
                    Rrw(2,0),Rrw(2,1),Rrw(2,2),trw(2),
                    0.f,0.f,0.f,1.f};

    return Trw;
}

cv::Matx31f KeyFrame::GetRightCameraCenter_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rwl = Tcw_.get_minor<3,3>(0,0).t();
    cv::Matx31f tlr = Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f twr = Rwl * tlr + Ow_;

    return twr;
}

cv::Matx31f KeyFrame::UnprojectStereo_(int i) {
    const float z = mvDepth->at(i);//const float z = mvDepth[i];
    if(z>0)
    {
        const float u = (*mvKeys)[i].pt.x;
        const float v = (*mvKeys)[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Matx31f x3Dc(x,y,z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc_.get_minor<3,3>(0,0) * x3Dc + Twc_.get_minor<3,1>(0,3);
    }
    else
        return cv::Matx31f::zeros();
}

cv::Matx44f KeyFrame::GetPose_()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_;
}



} //namespace ORB_SLAM
