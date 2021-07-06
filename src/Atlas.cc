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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    //mpCurrentMap = static_cast<boost::interprocess::offset_ptr<Map> >(NULL);
    mpCurrentMap = 0;
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{

    std::cout<<"Atlas initialized:"<<std::endl;
     

    //mpCurrentMap = static_cast<boost::interprocess::offset_ptr<Map> >(NULL);
    mpCurrentMap = 0;
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<boost::interprocess::offset_ptr<Map> >::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        boost::interprocess::offset_ptr<Map>  pMi = *it;

        if(pMi)
        {
            delete pMi.get();
            pMi = static_cast<boost::interprocess::offset_ptr<Map> >(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    cout<<"In create New Map()"<<endl;
    unique_lock<mutex> lock(mMutexAtlas);
    //cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    //mpCurrentMap = new Map(mnLastInitKFidMap);


    //Aditya: Use shared memory segment to create a new object, give a name with its map ID
    //Open managed shared memory
    boost::interprocess::managed_shared_memory segment_mem(boost::interprocess::open_or_create, "MySharedMemory",10737418240);
    //std::string name_map = "Map";
    segment = &segment_mem;
    //std::string string_num_map = to_string(mnLastInitKFidMap);
    //name_map.append(string_num_map);

    //initialize the map now.

    mpCurrentMap = segment->find_or_construct<Map>("Map1") (mnLastInitKFidMap);
    cout<<"Created Map object in shared memory! Address is: "<<mpCurrentMap<<endl;
    cout<<"Reading a variable there "<<mpCurrentMap->GetMaxKFid()<<endl;

    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(boost::interprocess::offset_ptr<Map>  pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Chage to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap.get();
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(boost::interprocess::offset_ptr<KeyFrame>  pKF)
{
    boost::interprocess::offset_ptr<Map>  pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(boost::interprocess::offset_ptr<MapPoint>  pMP)
{
    boost::interprocess::offset_ptr<Map>  pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<boost::interprocess::offset_ptr<KeyFrame> > Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<boost::interprocess::offset_ptr<MapPoint> > Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<boost::interprocess::offset_ptr<MapPoint> > Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<boost::interprocess::offset_ptr<Map> > Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(boost::interprocess::offset_ptr<Map>  elem1 ,boost::interprocess::offset_ptr<Map>  elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<boost::interprocess::offset_ptr<Map> > vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<boost::interprocess::offset_ptr<Map> >::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    //mpCurrentMap = static_cast<boost::interprocess::offset_ptr<Map> >(NULL);
    //Aditya
    mpCurrentMap = NULL;
    mnLastInitKFidMap = 0;
}

//boost::interprocess::offset_ptr<Map>  Atlas::GetCurrentMap()
Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();

    //Open managed shared memory
    
    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create, "MySharedMemory",10737418240);
    
    currentMapName = "Map1";
    cout<<"Checked if new map is required. Runing a function in Shared memory"<<endl;
    mpCurrentMap = segment.find_or_construct<Map>("Map1")();

    
    //run functions.
    cout<<"GetId: "<<mpCurrentMap->GetId()<<endl;
    cout<<"Address of mpCurrentMap "<<mpCurrentMap<<endl;

    //let's see if we can access mpCurrentMap
    cout<<"mpCurrentMap->nNextId: "<<mpCurrentMap->nNextId<<endl;
    //check boolean
    /*
    if(mpCurrentMap->mbBad)
        cout<<"mbBad is true"<<endl;
    else
        cout<<"mbBad is false"<<endl;

    */
    while(mpCurrentMap->IsBad())
        usleep(3000);


    return mpCurrentMap;
}

void Atlas::SetMapBad(boost::interprocess::offset_ptr<Map>  pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(boost::interprocess::offset_ptr<Map>  pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<boost::interprocess::offset_ptr<Map> >(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    cout<<"Address of mpCurrentMap "<<mpCurrentMap<<endl;
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(boost::interprocess::offset_ptr<Map>  mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (boost::interprocess::offset_ptr<Map> mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

} //namespace ORB_SLAM3
