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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <string>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>


namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

class Atlas
{

public:
    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
    ~Atlas();

    void CreateNewMap();
    void ChangeMap(boost::interprocess::offset_ptr<Map>  pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(boost::interprocess::offset_ptr<KeyFrame>  pKF);
    void AddMapPoint(boost::interprocess::offset_ptr<MapPoint>  pMP);

    void AddCamera(GeometricCamera* pCam);

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > GetAllKeyFrames();
    std::vector<boost::interprocess::offset_ptr<MapPoint> > GetAllMapPoints();
    std::vector<boost::interprocess::offset_ptr<MapPoint> > GetReferenceMapPoints();

    vector<boost::interprocess::offset_ptr<Map> > GetAllMaps();

    int CountMaps();

    void clearMap();

    void clearAtlas();

    boost::interprocess::offset_ptr<Map>  GetCurrentMap();
    //Map*  GetCurrentMap();
    boost::interprocess::offset_ptr<Map> currentMapPtr;

    void SetMapBad(boost::interprocess::offset_ptr<Map>  pMap);
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();
    string currentMapName;

    int sum_of_two();

    //example
     int a;
    int b;

    //for managed shared memory converted
    boost::interprocess::fixed_managed_shared_memory *segment;
    //boost::interprocess::managed_shared_memory *segment;

protected:

    std::set<boost::interprocess::offset_ptr<Map> > mspMaps;
    std::set<boost::interprocess::offset_ptr<Map> > mspBadMaps;
    //boost::interprocess::offset_ptr<Map>  mpCurrentMap;
    Map* mpCurrentMap;

    std::vector<GeometricCamera*> mvpCameras;
    std::vector<KannalaBrandt8*> mvpBackupCamKan;
    std::vector<Pinhole*> mvpBackupCamPin;

    std::mutex mMutexAtlas;

    unsigned long int mnLastInitKFidMap;

    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    



}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
