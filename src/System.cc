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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>


namespace ORB_SLAM3
{
   
    //Shared memory variables
    //boost::interprocess::fixed_managed_shared_memory segment(boost::interprocess::open_or_create, "MySharedMemory",10737418240,(void*)0x30000000);
    //boost::interprocess::fixed_managed_shared_memory segment(boost::interprocess::open_only, "MySharedMemory",(void*)0x300000000);
    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "MySharedMemory");//,(void*)0x30000000);
    //boost::interprocess::mapped_region region(segment, boost::interprocess::read_write);
    
    //Create an allocator that allocates ints from the managed segment
    //boost::interprocess::allocator<char, boost::interprocess::fixed_managed_shared_memory::segment_manager> allocator_instance(segment.get_segment_manager());
    boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> allocator_instance(segment.get_segment_manager());

    //cout<<"Installing Shared memory "<<endl;



Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence, const string &strLoadingFile):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)//,segment(boost::interprocess::open_or_create, "MySharedMemory",10737418240)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    
    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    bool loadedAtlas = false;

    //----
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    
    //boost::interprocess::fixed_managed_shared_memory seg(boost::interprocess::open_only, "MySharedMemory",(void*)0x30000000);

    //boost::interprocess::managed_shared_memory seg(boost::interprocess::open_only, "MySharedMemory");
    //int *magic2 = (seg.find<int>("magic-num2")).first;
    //std::cout<<"Magic num is :"<<(*magic2)<<std::endl;
    //segment = &seg;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    cout<<"Created Key frame database"<<endl;
    //first see if there is already a map or not
    std::pair<int *,std::size_t> ret = ORB_SLAM3::segment.find<int>("magic-num");
    int *magic_num = nullptr;

    if(ret.first == 0)
    {
        int newnum = 2;
        std::cout<<"First pointer is 0; First process"<<std::endl;
        magic_num = ORB_SLAM3::segment.construct<int>("magic-num")(newnum);
        *magic_num = newnum;
        std::cout<<"Made the magic-num memory. Value: "<<*magic_num<<std::endl;
        

    }
    else{
        std::cout<<"Not first process. Magic Num: "<<(*(ret.first))<<std::endl;
        magic_num = ret.first;
        

    }

    std::cout<<"Incrementing the magic number\n";
    *magic_num = *magic_num+1;
    std::cout<<"New magic number "<<*magic_num<<std::endl;
    //int *magic_num = ORB_SLAM3::segment.construct<int>("magic-num")(111, std::nothrow);
    
 
    //Creating a new atlas object in shared memory
    //Create the Atlas
    //mpAtlas = new Atlas(0);

    char atlasname[7];
    char otherAtlasname[7];

    atlasname[0] = 'a';
    atlasname[1] = 't';
    atlasname[2] = 'l';
    atlasname[3] = 'a';
    atlasname[4] = 's';
    atlasname[6] = '\0';

    memcpy(otherAtlasname,atlasname,sizeof(atlasname));

    sprintf(&atlasname[5],"%d",*magic_num);
    mpAtlas = (segment.find<Atlas>(atlasname)).first;

    //mpAtlas = (segment.find<Atlas>("Atlas")).first;
   
    
    if(0 == mpAtlas){
        std::cout<<"Atlas did not exist"<<std::endl;
        //mpAtlas = segment.construct<Atlas>("Atlas")(0);
        mpAtlas = segment.construct<Atlas>(atlasname)(*magic_num*200);
        sprintf(&otherAtlasname[5],"%d",(*magic_num)-1);
        KeyFrame::nNextId = *magic_num*200;
        MapPoint::nNextId = *magic_num*10000;
        Atlas* otherAtlas = (segment.find<Atlas>(otherAtlasname)).first;
        if(otherAtlas!=0){
            std::cout<<"Setting up reference points as points of other atlas\n";
            KeyFrame::nNextId = 1500;
            MapPoint::nNextId = 1500;

        //mpAtlas->SetReferenceMapPoints(otherAtlas->currentMapPtr->GetReferenceMapPoints());
    }
    }
    else{
        std::cout<<"Atlas EXISTED!! Using the same Atlas."<<std::endl;
    }
    mpAtlas->segment = &segment;
    //mpAtlas->processnum = *magic_num;
    //processnum = *magic_num;

    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;

    //let's look around the other atlas.. 
    cout<<" Output from Atlas: CountMaps() "<<mpAtlas->CountMaps()<<endl;
    
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, strSequence);
    //mpTracker = segment.find_or_construct<Tracking>("TrackingThread")(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
    //                         mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, strSequence);

    cout<<"mpTracker address in beginning: "<<mpTracker<<" Read integer:"<<mpTracker->mSensor<<" Read integer address: "<<&(mpTracker->mSensor)<<endl;

    offset_tracker = mpTracker;

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{

    //Open managed shared memory
    //boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create, "MySharedMemory",1073741824);

    //mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
    //                         mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, strSequence);
    //const string hello = "";
    //const string bye = "";

    //boost::interprocess::offset_ptr<Tracking> mpTracker_ptr = segment.find_or_construct<Tracking>("TrackingThread")(nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,hello,0,bye);
    //cout<<"Here.."<<mpTracker<<endl;
    //cout<<"name of managed shared memory: "<<boost::interprocess::managed_shared_memory::get_instance_name(mpTracker)<<endl;
    //cout<<"offset_tracker "<<mpTracker->mSensor<<endl;

    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            cout << "Reset stereo..." << endl;
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

  
    //cout<<"mpTracker address later: "<<mpTracker<<" offset of the pointer: "<<*((int*)((char*)mpTracker+30))<<endl;//" Read integer:"<<mpTracker->mSensor<<endl;


    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp,filename);

    
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }


    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    std::cout<<"Shutting down.... enter a number to continue shutdown\n";
    int aa;
    std::cin>>aa;
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;
        if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
}



void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    //list<ORB_SLAM3::boost::interprocess::offset_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<boost::interprocess::offset_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        boost::interprocess::offset_ptr<KeyFrame>  pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<boost::interprocess::offset_ptr<Map> > vpMaps = mpAtlas->GetAllMaps();
    boost::interprocess::offset_ptr<Map>  pBiggerMap;
    int numMaxKFs = 0;
    for(boost::interprocess::offset_ptr<Map>  pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<boost::interprocess::offset_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        boost::interprocess::offset_ptr<KeyFrame>  pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            continue;
        }

        Trw = Trw*pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat Tbw = pKF->mImuCalib.Tbc*(*lit)*Trw;
            cv::Mat Rwb = Tbw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twb = -Rwb*Tbw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(Rwb);
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        else
        {
            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(Rwc);
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }

    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}


void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<boost::interprocess::offset_ptr<Map> > vpMaps = mpAtlas->GetAllMaps();
    boost::interprocess::offset_ptr<Map>  pBiggerMap;
    int numMaxKFs = 0;
    for(boost::interprocess::offset_ptr<Map>  pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs = mpAtlas->GetAllKeyFrames();
    cout<<endl<<" [[[[{{{{ The number of Keyframes considered for output : "<<vpKFs.size()<<endl;
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<boost::interprocess::offset_ptr<KeyFrame> >::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        boost::interprocess::offset_ptr<KeyFrame>  pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<boost::interprocess::offset_ptr<MapPoint> > System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif


void System::PostLoad(){
    std::cout<<"---- Running PostLoad ----"<<std::endl;
    //first check if the atlas is first one or later one.
    std::pair<int *,std::size_t> ret = ORB_SLAM3::segment.find<int>("magic-num");
    int *magic_num = ret.first;
    Atlas *otherAtlas=nullptr;

    //now check. 3 is the starting process. 
    if (*magic_num > 3)
    {
        //second process
        int previous_num = *magic_num-1;
        char atlasname[7];

        atlasname[0] = 'a';
        atlasname[1] = 't';
        atlasname[2] = 'l';
        atlasname[3] = 'a';
        atlasname[4] = 's';
        atlasname[6] = '\0';

        sprintf(&atlasname[5],"%d",previous_num);
        otherAtlas = (segment.find<Atlas>(atlasname)).first;

        std::cout<<"Printing the details form another atlas inside PostLoad!!\n";

        //std::cout<<"Other atlas timestamp:-- "<<otherAtlas->currentMapPtr->GetOriginKF()->GetNumberMPs()<<"\n";
        std::cout<<"Num of mappoints to currentMapPtr in OTHER atlas: "<<otherAtlas->GetCurrentMap()->MapPointsInMap()<<std::endl;
        
        std::cout<<"Other atlas timestamp:-- "<<otherAtlas->GetCurrentMap()->GetOriginKF()->GetNumberMPs()<<"\n";
        //std::cout<<"This atlas timestamp:-- "<<mpAtlas->currentMapPtr->GetOriginKF()->GetNumberMPs()<<"\n";
        std::cout<<"Point distribution of Other Atlas: (mnFrameID) "<<std::endl;
        otherAtlas->GetCurrentMap()->GetOriginKF()->PrintPointDistribution();
        std::cout<<std::endl;
        
        std::cout<<"Num of mappoints to currentMapPtr in CURRENT atlas: "<<mpAtlas->GetCurrentMap()->MapPointsInMap()<<std::endl;

        //adding other atlas's map
        //std::cout<<"--- BEGIN MERGING --- Adding Another Atlas's Map \n";
        //std::cout<<"Number of Maps before adding another map: "<<mpAtlas->CountMaps()<<std::endl;
        //std::cout<<"Number of Keyframes before adding the maps: "<<mpAtlas->KeyFramesInMap()<<std::endl;
        //Now try to get the first process's map.
        //mpAtlas->AddMap(otherAtlas->currentMapPtr);
        //std::cout<<"Number of maps in the Atlas now: "<<mpAtlas->CountMaps()<<std::endl;

        //before changing map, go through each keyframe of the map and convert the matrices.
        std::vector<boost::interprocess::offset_ptr<KeyFrame> > allkeyframes = otherAtlas->GetCurrentMap()->GetAllKeyFrames();
        std::cout<<"Size of the vector before fixing: "<<allkeyframes.size()<<std::endl;
        std::vector<boost::interprocess::offset_ptr<KeyFrame> > thesekeyframes = mpAtlas->GetCurrentMap()->GetAllKeyFrames();



        std::cout<<"----- Test the accessible keyframes Bag of words ------\n";
        for(auto k:thesekeyframes){
            std::cout<<"Size of Bag of Words: "<<k->mBowVec.size()<<std::endl;
            std::cout<<"Size of Feature vectors: "<<k->mFeatVec.size()<<std::endl;
            std::cout<<"Size of the feature matrix. Hopefully it is same size: "<<k->mDescriptors.size()<<std::endl;
            std::cout<<"Size of the element of mDescriptors: "<<k->mDescriptors.elemSize()<<std::endl;
            std::cout<<"KeyFrame ID: "<<k->mnId<<std::endl;
        }

        auto stopped_map = mpAtlas->GetCurrentMap();
        std::cout<<" -------old map ID: "<<mpAtlas->GetCurrentMap()->GetId()<<std::endl;
        // create a new map.
        mpAtlas->CreateNewMap();
        std::cout<<"---- New Map ID: ----"<<mpAtlas->GetCurrentMap()->GetId()<<std::endl;
        std::cout<<"---- Number of Maps: -----"<<mpAtlas->CountMaps()<<std::endl;
        //this should shift the new map as current
        //need to initialize the keyframe 
        
        //for(auto& camera: mpAtlas->getCurrentCamera()){
        //    mpAtlas->AddCamera(camera);
        //}
        int pause;
        std::cout<<"************ Paused to see above Points ********************\n";
        cin>>pause;
        
         //have to fix all mappoints and then add new mappoints.
        std::cout<<"------ ====== Adding all the mappoints ------ ========\n";
        std::vector<boost::interprocess::offset_ptr<MapPoint> > allmappoints = otherAtlas->GetCurrentMap()->GetAllMapPoints();
        //fix all the mapppoints.
        for(auto& mapP: allmappoints){
            mapP->UpdateMap(mpAtlas->GetCurrentMap());
            mapP->FixMatrices();
            mpAtlas->GetCurrentMap()->AddMapPoint(mapP);
        }

        mpAtlas->SetReferenceMapPoints(otherAtlas->GetReferenceMapPoints);

        for(auto& keyf: allkeyframes){
            keyf->FixMatrices(keyf);
            keyf->ResetCamera(mpAtlas->getCurrentCamera());
            keyf->UpdateMap(mpAtlas->GetCurrentMap());
            keyf->SetORBVocabulary(mpAtlas->GetORBVocabulary());
            keyf->SetKeyFrameDatabase(mpAtlas->GetKeyFrameDatabase());

            
        }
        for(auto& keyf: allkeyframes){
        //we should fix the descriptors differently.
            //with ORB vocab from new map
            thesekeyframes[0]->FixBow(keyf,mpAtlas->GetORBVocabulary());

            mpAtlas->GetCurrentMap()->AddKeyFrame(keyf);
            //mpKeyFrameDatabase->add(keyf);

        }
        std::cout<<"First map keyframes: "<<allkeyframes.size()<<" Number of Keyframes after adding the map: "<<mpAtlas->KeyFramesInMap()<<std::endl;
        
        //for(auto& keyf: allkeyframes){
        //for(int ii = 0; ii<(allkeyframes.size()-1); ii++){
        //    mpAtlas->GetKeyFrameDatabase()->add(allkeyframes[ii]);
       // }
        std::cout<<"First map keyframes: "<<allkeyframes.size()<<" Number of Keyframes after adding the map: "<<mpAtlas->KeyFramesInMap()<<std::endl;
        mpAtlas->ChangeMap(stopped_map);

       /*

        for(auto& mapP: allmappoints){
            mapP->UpdateMap(mpAtlas->currentMapPtr);
            mpAtlas->currentMapPtr->AddMapPoint(mapP);
            mapP->FixMatrices();

        }

        mpAtlas->SetReferenceMapPoints(otherAtlas->currentMapPtr->GetReferenceMapPoints());

        //mpLoopCloser->NewDetectCommonRegions();
        std::cout<<"Number of MapPointsInMap: "<<mpAtlas->MapPointsInMap()<<std::endl;


        std::cout<<"-----======= === === Completely cleaning the KeyFrameDatabase\n";
        //mpKeyFrameDatabase->clearMap(mpAtlas->currentMapPtr);

        //for(auto& keyfra: thesekeyframes){
         //   mpKeyFrameDatabase->add(keyfra);
        //}

        std::cout<<"---- ===== adding the keyframe to new map ----- ==== \n";
        //the keyframes from the other atlas
        for(auto& keyf: allkeyframes){
            keyf->FixMatrices(keyf);
            keyf->ResetCamera(mpAtlas->getCurrentCamera());
            keyf->UpdateMap(mpAtlas->currentMapPtr);
            keyf->SetORBVocabulary(mpAtlas->GetORBVocabulary());
            keyf->SetKeyFrameDatabase(mpAtlas->GetKeyFrameDatabase());

            //we should fix the descriptors differently.
            //with ORB vocab from new map
            thesekeyframes.at(0)->FixBow(keyf,mpAtlas->GetORBVocabulary());

            //mpAtlas->currentMapPtr->AddKeyFrame(keyf);
            //mpKeyFrameDatabase->add(keyf);
        }
        std::cout<<"First map keyframes: "<<allkeyframes.size()<<" Number of Keyframes after adding the map: "<<mpAtlas->KeyFramesInMap()<<std::endl;

        
        int count_kf = 0;
        for(auto& keyf: allkeyframes){
            //std::cout<<"Print the pose inverse matrix size "<<keyf->GetPoseInverse().t()<<std::endl;
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~ mpLocalMapper working on keyframe: "<<keyf->mnId<<" ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            mpLocalMapper->InsertKeyFrame(keyf);
            count_kf++;
            int aaa;
            sleep(1);
            //if(count_kf>=10)
            //{
             //   std::cout<<"STOP... Pause... after 10 keyframes\n";
             //   cin>>aaa;
            //}

        }



        std::cout<<"Finished all keyframes in queue in local mapper~~~~~~~\n";
        
        std::cout<<"++++ Finished adding all the keyframes. Finished fixing all the matrices.\n";
        
        std::cout<<"Stop for keyframes in queue."<<mpLocalMapper->KeyframesInQueue()<<std::endl;
        while(mpLocalMapper->KeyframesInQueue())
        {
            usleep(1000);
        }
        std::cout<<"****** GlobalBundleAdjustemnt started\n";
        mpLoopCloser->RunGlobalBundleAdjustment(mpAtlas->currentMapPtr,600);
        std::cout<<"------ GlobalBundleAdjustemnt finished\n";

        

        

        
        std::cout<<"Adding the reference mappoints from older map:"<<std::endl;
        //mpAtlas->SetReferenceMapPoints(otherAtlas->currentMapPtr->GetReferenceMapPoints());
        //std::cout<<"Added the Reference Mappoints from Previous Map\n";

        mpAtlas->currentMapPtr->CheckEssentialGraph();

        //std::cout<<"Add the existing keyframes from map to loopcloser to just seer if it works\n";
        //std::vector<boost::interprocess::offset_ptr<KeyFrame> > vkf = otherAtlas->currentMapPtr->GetAllKeyFrames();
        //mpLoopCloser->RequestReset();
        //mpKeyFrameDatabase->clear();
        //std::cout<<"First update maps of all keyframes\n";
        //for(auto k : vkf){

            //cout<<"KeyFrame adding to KeyFrameDatabase: "<<k->mnId<<std::endl;
                    //cout << "------press enter to continue------vkf size: " << vkf.size() << endl;
                    //string tmp;
                    //getline(cin,tmp);
                    //while(true){
                            //unique_lock<mutex> lock(mpLoopCloser->passedCheckingMutex);
                            //if(!mpLoopCloser->passedChecking){
                            //mpKeyFrameDatabase->add(k);
                            
            //mpLoopCloser->InsertKeyFrame(k);
            //string tmp;
            //getline(cin,tmp);
                            
                            //mpLoopCloser->passedChecking = true;
                            //break;
                            //}
                        usleep(1000);
                    //}
        //}
                        */
        std::cout<<"============ Finished updating the mpLoopcloser ================"<<std::endl;

        //Details before pause:


        std::cout<<"Pause\n";
        int a;
        std::cin>>a;

        //now update the mpAtlas to newly read map
        //std::cout<<"Changing the map to other atlas.\n";
        //mpAtlas->ChangeMap(otherAtlas->currentMapPtr);


    } 
    else{
        int previous_num = *magic_num;
        char atlasname[7];

        atlasname[0] = 'a';
        atlasname[1] = 't';
        atlasname[2] = 'l';
        atlasname[3] = 'a';
        atlasname[4] = 's';
        atlasname[6] = '\0';

        sprintf(&atlasname[5],"%d",previous_num);
        otherAtlas = (segment.find<Atlas>(atlasname)).first;
        std::cout<<"--- Still in First process. No need to merge\n";
        // print the number of keyframes.
        std::cout<<"Point distribution of This Atlas:(mnFrameID) "<<std::endl;//otherAtlas->currentMapPtr->GetOriginKF()->PrintPointDistribution()<<std::endl;
        
        //otherAtlas->currentMapPtr->GetOriginKF()->PrintPointDistribution();
        std::cout<<std::endl;
        //std::cout<<"Num of mappoints to currentMapPtr in THIS atlas: "<<otherAtlas->currentMapPtr->MapPointsInMap()<<std::endl;
    }   //std::cout<<"Other atlas timestamp:** "<<otherAtlas->currentMapPtr->GetOriginKF()->GetNumberMPs()<<"\n";

}


} //namespace ORB_SLAM


