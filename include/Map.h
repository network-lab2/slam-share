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


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

//Some Boost interprocess libarries.
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/set.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/serialization/base_object.hpp>

//using namespace boost::interprocess;


namespace ORB_SLAM3
{


class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;
class GeometricCamera;

class Map
{

public:
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(boost::interprocess::offset_ptr<KeyFrame> pKF);
    void AddMapPoint(boost::interprocess::offset_ptr<MapPoint> pMP);
    void EraseMapPoint(boost::interprocess::offset_ptr<MapPoint> pMP);
    void EraseKeyFrame(boost::interprocess::offset_ptr<KeyFrame> pKF);
    void SetReferenceMapPoints(const std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<boost::interprocess::offset_ptr<KeyFrame> > GetAllKeyFrames();
    std::vector<boost::interprocess::offset_ptr<MapPoint> > GetAllMapPoints();
    std::vector<boost::interprocess::offset_ptr<MapPoint> > GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    boost::interprocess::offset_ptr<KeyFrame> GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();
    int sum_of_two();

    //create your own vector!
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<KeyFrame>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator; 
    //typedef vector
    typedef vector<boost::interprocess::offset_ptr<KeyFrame>, ShmemAllocator> MyVector;
    //changed the vector //old code
    //vector<boost::interprocess::offset_ptr<KeyFrame> > mvpKeyFrameOrigins;
    MyVector *mvpKeyFrameOrigins;
    boost::interprocess::offset_ptr<MyVector> keyframeorigins_offsetptr;


    //old code
    //vector<unsigned long int> mvBackupKeyFrameOriginsId;
    typedef boost::interprocess::allocator<unsigned long int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_longint; 
    typedef vector<unsigned long int, ShmemAllocator_longint> MyVector_longint;
    MyVector_longint *mvBackupKeyFrameOriginsId;
    

    boost::interprocess::offset_ptr<KeyFrame> mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    /* operators aditya added
    void * operator new(size_t);
    void operator delete(void *);
    */

    //for managed shared memory
    //boost::interprocess::fixed_managed_shared_memory *segment;
    boost::interprocess::managed_shared_memory *segment;

    //bool mbBad = false;

std::mutex mMutexMap;
boost::interprocess::offset_ptr<std::mutex> mMutexMapPtr;

 bool mbImuInitialized;

 //example
 int a;
 int b;

 //create your own vector!
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<MapPoint>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_mappoint; 
    //typedef vector
    typedef vector<boost::interprocess::offset_ptr<MapPoint>, ShmemAllocator_mappoint> MyVector_mappoint;
protected:

    long unsigned int mnId;

    //new code for Set -mappoints 
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<MapPoint>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_mappoint_set;
    typedef boost::interprocess::set<boost::interprocess::offset_ptr<MapPoint>, std::less<boost::interprocess::offset_ptr<MapPoint> >,ShmemAllocator_mappoint_set> Myset_mappoint;


    //new code for Set.
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<KeyFrame>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_keyframe_set;
    typedef boost::interprocess::set<boost::interprocess::offset_ptr<KeyFrame>, std::less<boost::interprocess::offset_ptr<KeyFrame> >,ShmemAllocator_keyframe_set> Myset_keyframe;

    // Old-code
    //std::set<boost::interprocess::offset_ptr<MapPoint> > mspMapPoints;
    // New-Code
    boost::interprocess::offset_ptr<Myset_mappoint> mspMapPoints;
    std::set<boost::interprocess::offset_ptr<MapPoint> > mspMapPoints_support;
    
    //std::set<boost::interprocess::offset_ptr<KeyFrame> > mspKeyFrames;
    // New-code
    boost::interprocess::offset_ptr<Myset_keyframe> mspKeyFrames;
    std::set<boost::interprocess::offset_ptr<KeyFrame> > mspKeyFrames_support;


    boost::interprocess::offset_ptr<KeyFrame> mpKFinitial;
    boost::interprocess::offset_ptr<KeyFrame> mpKFlowerID;

    //old-code
    //std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpReferenceMapPoints;
    boost::interprocess::offset_ptr<MyVector_mappoint> mvpReferenceMapPoints;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpReferenceMapPoints_support;

   

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool mbBad;

    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
  
    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    
};

} //namespace ORB_SLAM3

#endif // MAP_H
